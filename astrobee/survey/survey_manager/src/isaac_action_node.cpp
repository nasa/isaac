/* Copyright (c) 2021, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
 * platform" software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <ros/ros.h>

#include <plansys2_executor/ActionExecutorClient.hpp>

#include <stdio.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <iostream>
#include <string>
#include <vector>

#include "survey_manager/isaac_action_node.h"

namespace plansys2_actions {

// Class for running commands and collecting their stdout output, roughly following Python
// subprocess.Popen but focused on one special case.
class PopenReader {
 public:
  // Start running `cmd` using /bin/sh, inheriting the current process's environment variables. An
  // internal reader pipe will be used to collect its stdout output.
  explicit PopenReader(const char* cmd) : reader_(nullptr), child_pid_(-1), child_status_(127) {
    int pipe_fds[2];
    if (pipe(pipe_fds) < 0) {
      return;
    }
    const char* argv[4];
    argv[0] = "sh";
    argv[1] = "-c";
    argv[2] = cmd;
    argv[3] = nullptr;

    int pid;
    switch (pid = fork()) {
      case -1:  // fork error
        close(pipe_fds[0]);
        close(pipe_fds[1]);
        return;
      case 0:  // child
        if (pipe_fds[1] != STDOUT_FILENO) {
          dup2(pipe_fds[1], STDOUT_FILENO);
        }
        {
          int fdlimit = static_cast<int>(sysconf(_SC_OPEN_MAX));
          for (int i = STDERR_FILENO + 1; i < fdlimit; i++) {
            close(i);
          }
        }
        execve("/bin/sh", (char* const*)argv, environ);
        exit(127);  // execve() error, exit child
      default:      // parent
        close(pipe_fds[1]);
        reader_ = fdopen(pipe_fds[0], "r");
        child_pid_ = pid;
    }
  }

  // Collect stdout output from running child process and wait for it to terminate. Does
  // nothing if reader pipe wasn't successfully opened.
  void communicate() {
    if (reader_ == nullptr) {
      return;
    }

    std::array<char, 128> chunk;
    while (fgets(chunk.data(), chunk.size(), reader_) != nullptr) {
      stdout_ += chunk.data();
    }
    fclose(reader_);

    int wait_result;
    int status;
    do {
      wait_result = wait4(child_pid_, &status, 0, nullptr);
    } while (wait_result == -1 && errno == EINTR);
    child_status_ = status;
  }

  // Run `cmd` using /bin/sh, inheriting the current process's environment variables, wait for child
  // to terminate, and return its status. Set `stdout` to child's stdout output, if any.  Return 127
  // in case of internal failures prior to starting child. Roughly similar to Python
  // subprocess.getstatusoutput().
  static int get_status_output(const char* cmd, std::string& stdout) {
    PopenReader proc(cmd);
    proc.communicate();
    stdout = proc.stdout_;
    return proc.child_status_;
  }

 protected:
  FILE* reader_;
  pid_t child_pid_;
  int child_status_;
  std::string stdout_;
};

// Return the estimated duration (seconds) of action `action_name`. Queried from PDDL model. On
// query error, return a default value of one minute.
double get_action_duration(const std::string& action_name) {
  std::string cmd = std::string("rosrun survey_manager pddl_query action_duration ") + action_name;
  printf("get_action_duration %s: running query: %s\n", action_name.c_str(), cmd.c_str());

  const double default_duration = 60.0;  // one minute
  std::string duration_buf;
  int query_status = PopenReader::get_status_output(cmd.c_str(), duration_buf);

  if (query_status != EXIT_SUCCESS) {
    printf("get_action_duration %s: query returned with non-zero exit code %d, using default duration of %.1lf",
           action_name.c_str(), query_status, default_duration);
    return default_duration;
  }

  char* ending;
  double duration = strtod(duration_buf.c_str(), &ending);

  if (ending == duration_buf.c_str()) {
    printf("get_action_duration %s: couldn't parse query output '%s', using default duration of %.1lf\n",
           action_name.c_str(), duration_buf.c_str(), default_duration);
    return default_duration;
  }

  printf("get_action_duration %s: estimated duration is %.1lf\n", action_name.c_str(), duration);
  return duration;
}

IsaacAction::IsaacAction(ros::NodeHandle nh, const std::string& action, const std::chrono::nanoseconds& rate,
                         bool quick)
    : ActionExecutorClient(nh, action, rate), quick_(quick) {
  action_name_ = action;
  progress_ = 0.0;
  pid_ = 0;
  command_ = "";

  // Get estimated action duration
  action_duration_ = get_action_duration(action_name_);
}

void IsaacAction::do_work() {
  std::string from, towards;

  // Start process if not started yet
  if (progress_ == 0.0) {
    const std::vector<std::string>& command_args = get_arguments();
    if (command_args.size() < 3) {
      finish(false, 1.0, "Not enough arguments for [MOVE] command");
    }

    std::string args_str = action_name_;
    for (auto arg : command_args) {
      args_str += " " + arg;
    }
    command_ = std::string("(") + args_str + ")";
    std::string quick_str = quick_ ? "--quick " : "";
    std::string command_astrobee_call = std::string("rosrun survey_manager command_astrobee ") + quick_str + args_str;

    start_time_ = ros::Time::now();
    pid_ = fork();
    if (pid_ < 0) {
      perror("isaac_action_node: Fork failed");
      finish(false, 1.0, "Failed to start the process");
    } else if (pid_ == 0) {  // child
      const char* args[4];
      args[0] = "sh";
      args[1] = "-c";
      args[2] = command_astrobee_call.c_str();
      args[3] = NULL;
      printf("isaac_action_node: Running: %s\n", args[2]);
      execvpe("sh", (char* const*)args, environ);
      perror("isaac_action_node: Failed to execute command");
      printf("isaac_action_node: %s: EXITING FAILURE %d\n", command_.c_str(), getpid());
      exit(-1);
    } else {
      progress_ = 0.02;
      return;
    }
  }

  // Note: This progress metric can intentionally exceed 1.0 if the action takes longer than
  // expected. This is useful because continued changes in progress verify that the action node is
  // still ticking, and the operator can tell at a glance that the action is running long.
  // (Hopefully it doesn't violate any assumptions in PlanSys2.)
  progress_ = (ros::Time::now() - start_time_).toSec() / action_duration_;
  send_feedback(progress_, command_ + " running");

  // Status gets printed on terminal
  // printf("\t ** %s [%5.1f%%]  \n", command_.c_str(), progress_ * 100.0);=
  int status;
  int result = waitpid(-1, &status, WNOHANG);
  // printf("Result: %d %d %d\n", result, pid_, status);
  if (result < 0) {
    perror("isaac_action_node: Failed to wait for pid");
    progress_ = 0.0;
    finish(false, 1.0, "Unexpected error waiting for process.");
  } else if (result == pid_) {
    if (status == 0) {
      std::cout << "isaac_action_node: " << command_ << ": Success " << std::endl;
      progress_ = 0.0;
      finish(true, 1.0, command_ + " completed");
    } else {
      std::cout << "isaac_action_node: " << command_ << " : Command terminated with non-zero exit code:  " << status
                << std::endl;
      progress_ = 0.0;
      finish(false, 1.0, command_ + " terminated by signal");
    }
  }
}

IsaacAction::~IsaacAction() {
  if (pid_ != 0) {
    // Kill the child process
    kill(pid_, SIGKILL);
  }
}
}  // namespace plansys2_actions


// Main entry point for application
int isaac_action_main(int argc, char *argv[], const char* action_name) {
  // Initialize a ros node
  ros::init(argc, argv, (std::string(action_name) + "_action").c_str());

  bool quick = false;
  if (argc == 2 && std::string(argv[1]) == "--quick") {
    printf("isaac_action_node[%s]: Got --quick; running longer actions in quick mode\n", action_name);
    quick = true;
  }

  std::string name = ros::this_node::getName();
  if (name.empty() || (name.size() == 1 && name[0] == '/'))
    name = "default";
  else if (name[0] == '/')
    name = name.substr(1);

  ros::NodeHandle nh("~");
  nh.setParam("action_name", action_name);

  // Start action node
  // We could actually add multiple action nodes here being aware that we might need a ros::AsyncSpinner
  // (https://github.com/Bckempa/ros2_planning_system/blob/noetic-devel/plansys2_bt_actions/src/bt_action_node.cpp#L41)
  auto action_node = std::make_shared<plansys2_actions::IsaacAction>(nh, action_name,  std::chrono::seconds(2), quick);
  action_node->trigger_transition(ros::lifecycle::CONFIGURE);

  // Synchronous mode
  ros::spin();
  // Make for great success
  return 0;
}
