/*
 * wifi-scan library header
 *
 * Copyright 2016-2018 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 */

#ifndef WIFI_WIFI_H_
#define WIFI_WIFI_H_

// Standard includes

#include <libmnl/libmnl.h>    // netlink libmnl
#include <linux/nl80211.h>    // nl80211 netlink
#include <linux/genetlink.h>  // generic netlink

#include <sys/socket.h>
#include <net/if.h>
#include <string.h>
#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>
#include <fcntl.h>            // fntnl (set descriptor options)
#include <errno.h>            // errno

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <string>

/**
 * \ingroup hw
 */
namespace wifi {

// some constants - mac address length, mac adress string length, max length of wireless network id with null character
enum wifi_constants {
  BSSID_LENGTH              = 6,
  BSSID_STRING_LENGTH       = 18,
  SSID_MAX_LENGTH_WITH_NULL = 33
};

// anything >=0 should mean that your are associated with the station
enum bss_status {
  BSS_NONE           = -1,
  BSS_AUTHENTHICATED = 0,
  BSS_ASSOCIATED     = 1,
  BSS_IBSS_JOINED    = 2
};



// a single wireless network can have multiple BSSes working as network under one SSID
struct bss_info {
  uint8_t bssid[BSSID_LENGTH];           // this is hardware mac address of your AP
  uint32_t frequency;                    // this is AP frequency in mHz
  char ssid[SSID_MAX_LENGTH_WITH_NULL];  // this is the name of your AP as you see it when connecting
  enum bss_status status;                // anything >=0 means that your are connected to this station/network
  int32_t signal_mbm;                    // signal strength in mBm, divide it by 100 to get signal in dBm
  int32_t seen_ms_ago;                   // when the above information was collected
};


// like above
struct station_info {
  uint8_t bssid[BSSID_LENGTH];           // this is hardware mac address of your AP
  char ssid[SSID_MAX_LENGTH_WITH_NULL];  // this is the name of your AP as you see it when connecting
  enum bss_status status;                // anything >=0 means that your are connected to this station/network
  int8_t signal_dbm;                     // signal strength in dBm from last received PPDU
  int8_t signal_avg_dbm;                 // signal strength average in dBm
  uint32_t rx_packets;                   // the number of received packets
  uint32_t tx_packets;                   // the number of transmitted packets
};


class Wifi {
 public:
  // Constructor
  Wifi();

  /* Initializes the library
   *
   * If this functions fails the library will die with error message explaining why
   *
   * parameters:
   * interface - wireless interface, e.g. wlan0, wlan1
   *
   */
  int WifiScanInit(const char *interface);

  /* Frees the resources used by library
   *
   */
  void WifiScanClose();

  /* Retrieve information about station you are associated to
   *
   * Retrieves information only about single station.
   * This function can be called repeateadly fast.
   *
   * parameters:
   * station - to be filled with information
   *
   * returns:
   * -1 on error (errno is set), 0 if not associated to any station, 1 if data was retrieved
   *
   * preconditions:
   * wifi initialized with wifi_scan_init
   *
   */
  int WifiScanStation(struct station_info *station);

  /* Make a passive scan of all networks around.
   *
   * This function triggers passive scan if necessery, waits for completion and returns the data.
   * If some other scan was triggered in the meanwhile the library will collect it's results.
   * Triggering a scan requires permissions, for testing you may use sudo.
   *
   * Scanning may take some time (it can be order of second).
   * While scanning the link may be unusable for other programs!
   *
   * parameters:
   * bss_infos - array of bss_info of size bss_infos_length
   * bss_infos_length - the length of passed array
   *
   * returns:
   * -1 on error (errno is set) or the number of found BSSes, the number may be greater then bss_infos_length
   *
   * Some devices may fail with -1 and errno=EBUSY if triggering scan when another scan is in progress. You may wait and retry in that case 
   *
   * preconditions:
   * wifi initialized with wifi_scan_init
   *
   */
  int WifiScanAll(struct bss_info *bss_infos, int bss_infos_length);

  // convert bssid to printable hardware mac address
  std::string bssid_to_string(const uint8_t bssid[BSSID_LENGTH]) {
    char bssid_buff[BSSID_STRING_LENGTH];
    snprintf(bssid_buff, BSSID_STRING_LENGTH, "%02x:%02x:%02x:%02x:%02x:%02x",
           bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);\
    return bssid_buff;
  }

 private:
  // everything needed for sending/receiving with netlink
  struct netlink_channel {
    struct mnl_socket *nl;  // netlink socket
    char *buf;              // buffer for messages (in and out)
    uint16_t nl80211_id;    // generic netlink nl80211 id
    uint32_t ifindex;       // the wireless interface number (e.g. interface number for wlan0)
    uint32_t sequence;      // the sequence number of netlink message
    void *context;          // additional data to be stored/used when processing concrete message
  };

  // internal library data passed around by user
  struct wifi_scan {
     struct netlink_channel notification_channel;
     struct netlink_channel command_channel;
  };
  // DECLARATIONS AND TOP-DOWN LIBRARY OVERVIEW

  // INITIALIZATION

  // data needed from CTRL_CMD_GETFAMILY for nl80211, nl80211 id is stored in the channel rather then here
  struct context_CTRL_CMD_GETFAMILY {
    uint32_t id_NL80211_MULTICAST_GROUP_SCAN;  // the id of group scan which we need to subscribe to
  };

  // allocate memory, set initial values, etc.
  static int InitNetlinkChannel(struct netlink_channel *channel, const char *interface);
  // execute command to get nl80211 family and process the results
  static int GetFamilyAndScanIds(struct netlink_channel *channel);
  // this processes kernel reply for get family request, stores family id
  static int handle_CTRL_CMD_GETFAMILY(const struct nlmsghdr *nlh, void *data);
  // parses multicast groups to get scan multicast group id
  static void parse_CTRL_ATTR_MCAST_GROUPS(struct nlattr *nested, struct netlink_channel *channel);

  // subscribes channel to multicast group scan using scan group id
  static void subscribe_NL80211_MULTICAST_GROUP_SCAN(struct netlink_channel *channel, uint32_t scan_group_id);

  // CLEANUP

  // cleans up after single channel
  static void CloseNetlinkChannel(struct netlink_channel *channel);

  // SCANNING
  // SCANNING - notification related

  // the data needed from notifications
  struct context_NL80211_MULTICAST_GROUP_SCAN {
    int new_scan_results;  // are new scan results waiting for us?
    int scan_triggered;    // was scan was already triggered by somebody else?
  };

  // read but do not block
  static int ReadPastNotifications(struct netlink_channel *notifications);
  // this handles notifications
  static int handle_NL80211_MULTICAST_GROUP_SCAN(const struct nlmsghdr *nlh, void *data);
  // triggers scan if no results are waiting yet and if it was not already triggered
  static int TriggerScanIfNecessary(struct netlink_channel *commands,
                                       struct context_NL80211_MULTICAST_GROUP_SCAN *scanning);
  // wait for the notification that scan finished
  static int WaitForNewScanResults(struct netlink_channel *notifications);

  // // SCANNING - scan related

  // the data needed from new scan results
  struct context_NL80211_CMD_NEW_SCAN_RESULTS {
    struct bss_info *bss_infos;
    int bss_infos_length;
    int scanned;
  };

  // get scan results cached by the driver
  static int GetScan(struct netlink_channel *channel);
  // process the new scan results
  static int handle_NL80211_CMD_NEW_SCAN_RESULTS(const struct nlmsghdr *nlh, void *data);
  // get the information about bss (nested attribute)
  static void parse_NL80211_ATTR_BSS(struct nlattr *nested, struct netlink_channel *channel);
  // get the information from IE (non-netlink binary data here!)
  static void parse_NL80211_BSS_INFORMATION_ELEMENTS(struct nlattr *attr, char SSID_OUT[33]);
  // get BSSID (mac address)
  static void parse_NL80211_BSS_BSSID(struct nlattr *attr, uint8_t bssid_out[BSSID_LENGTH]);

  // STATION

  // data needed from command new station
  struct context_NL80211_CMD_NEW_STATION {
    struct station_info *station;
  };

  // get information about station with BSSID
  static int GetStation(struct netlink_channel *channel, uint8_t bssid[BSSID_LENGTH]);
  // process command new station
  static int handle_NL80211_CMD_NEW_STATION(const struct nlmsghdr *nlh, void *data);
  // process station info (nested attribute)
  static void parse_NL80211_ATTR_STA_INFO(struct nlattr *nested, struct netlink_channel *channel);

  // NETLINK HELPERS

  // NETLINK HELPERS - message construction/sending/receiving

  // create the message with specified parameters for the channel
  // fill the message with additional attributes as needed with:
  // mnl_attr_put_[|u8|u16|u32|u64|str|strz] and mnl_attr_nest_[start|end]
  static struct nlmsghdr *PrepareNlMessage(uint32_t type, uint16_t flags,
                                           uint8_t genl_cmd, struct netlink_channel *channel);
  // send the above message
  static void SendNlMessage(struct nlmsghdr *nlh, struct netlink_channel *channel);
  // receive the results and process them using callback function
  static int ReceiveNlMessage(struct netlink_channel *channel, mnl_cb_t callback);

  // NETLINK HELPERS - validation

  // all information needed to validate attributes
  struct validation_data {
    // validated attributes are returned here
    struct nlattr **attribute_table;
    // at most that many, distinct constants from nl80211.h go here
    int attribute_length;
    // vavildate against that table
    const struct attribute_validation *validation;
    int validation_length;
  };

  // data of type struct validation_data*, validate attr against data, this is called for each attribute
  static int Validate(const struct nlattr *attr, void *data);

  // #####################################################################
  // IMPLEMENTATION

  struct wifi_scan *wifi_;
};
  // NETLINK HELPERS - validation

  // formal requirements for attribute
  struct attribute_validation {
    // attribute constant from nl80211.h
    int attr;
    // MNL_TYPE_[UNSPEC|U8|U16|U32|U64|STRING|FLAG|MSECS|NESTED|NESTED_COMPAT|NUL_STRING|BINARY]
    enum mnl_attr_data_type type;
    // length in bytes, can be ommitted for attibutes of known size (e.g. U16), can be 0 if unspeciffied
    size_t len;
  };
  // validate only what we are going to use, note that
  // this lists all the attributes used by the library

  static constexpr struct attribute_validation NL80211_VALIDATION[2] = {
      {CTRL_ATTR_FAMILY_ID, MNL_TYPE_U16},
      {CTRL_ATTR_MCAST_GROUPS, MNL_TYPE_NESTED}
    };

  static constexpr struct attribute_validation NL80211_MCAST_GROUPS_VALIDATION[2] = {
      {CTRL_ATTR_MCAST_GRP_ID, MNL_TYPE_U32},
      {CTRL_ATTR_MCAST_GRP_NAME, MNL_TYPE_STRING}
    };

  static constexpr struct attribute_validation NL80211_BSS_VALIDATION[6] = {
      {NL80211_BSS_BSSID, MNL_TYPE_BINARY, 6},
      {NL80211_BSS_FREQUENCY, MNL_TYPE_U32},
      {NL80211_BSS_INFORMATION_ELEMENTS, MNL_TYPE_BINARY},
      {NL80211_BSS_STATUS, MNL_TYPE_U32},
      {NL80211_BSS_SIGNAL_MBM, MNL_TYPE_U32},
      {NL80211_BSS_SEEN_MS_AGO, MNL_TYPE_U32}
    };

  static constexpr struct attribute_validation NL80211_NEW_SCAN_RESULTS_VALIDATION[3] = {
      {NL80211_ATTR_IFINDEX, MNL_TYPE_U32},
      {NL80211_ATTR_SCAN_SSIDS, MNL_TYPE_NESTED},
      {NL80211_ATTR_BSS, MNL_TYPE_NESTED}
    };

  static constexpr struct attribute_validation NL80211_CMD_NEW_STATION_VALIDATION[1] = {
      {NL80211_ATTR_STA_INFO, MNL_TYPE_NESTED}
    };

  static constexpr struct attribute_validation NL80211_STA_INFO_VALIDATION[4] = {
      {NL80211_STA_INFO_SIGNAL, MNL_TYPE_U8},
      {NL80211_STA_INFO_SIGNAL_AVG, MNL_TYPE_U8},
      {NL80211_STA_INFO_RX_PACKETS, MNL_TYPE_U32},
      {NL80211_STA_INFO_TX_PACKETS, MNL_TYPE_U32}
    };

  static constexpr int NL80211_VALIDATION_LENGTH
                = sizeof(NL80211_VALIDATION)/sizeof(struct attribute_validation);
  static constexpr int NL80211_MCAST_GROUPS_VALIDATION_LENGTH
                = sizeof(NL80211_MCAST_GROUPS_VALIDATION)/sizeof(struct attribute_validation);
  static constexpr int NL80211_BSS_VALIDATION_LENGTH
                = sizeof(NL80211_BSS_VALIDATION)/sizeof(struct attribute_validation);
  static constexpr int NL80211_NEW_SCAN_RESULTS_VALIDATION_LENGTH
                = sizeof(NL80211_NEW_SCAN_RESULTS_VALIDATION)/sizeof(struct attribute_validation);
  static constexpr int NL80211_CMD_NEW_STATION_VALIDATION_LENGTH
                = sizeof(NL80211_CMD_NEW_STATION_VALIDATION)/sizeof(struct attribute_validation);
  static constexpr int NL80211_STA_INFO_VALIDATION_LENGTH
                = sizeof(NL80211_STA_INFO_VALIDATION)/sizeof(struct attribute_validation);


}  // namespace wifi
#endif  // WIFI_WIFI_H_
