/*
 * wifi-scan library implementation
 *
 * Copyright 2016-2018 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 */

#include <wifi/wifi.h>

namespace wifi {


// By default the arm is uninitialized
  Wifi::Wifi() {}

// #####################################################################
// IMPLEMENTATION


// INITIALIZATION

// public interface - pass wireless interface like wlan0
int Wifi::WifiScanInit(const char *interface) {
  wifi_ = (struct wifi_scan *) malloc(sizeof(struct wifi_scan));
  if (wifi_ == NULL) {
    ROS_ERROR("Insufficient memory - malloc(sizeof(struct wifi_data)");
    return -1;
  }

  InitNetlinkChannel(&wifi_->notification_channel, interface);

  struct context_CTRL_CMD_GETFAMILY family_context = {0};
  wifi_->notification_channel.context = &family_context;

  // create netlink sockets for generic netlink
  if (GetFamilyAndScanIds(&wifi_->notification_channel) == -1) {
    ROS_ERROR("GetFamilyAndScanId failed");
    return -1;
  }

  if (family_context.id_NL80211_MULTICAST_GROUP_SCAN == 0) {
    ROS_ERROR("No scan multicast group in generic netlink nl80211\n");
    return -1;
  }

  if (InitNetlinkChannel(&wifi_->command_channel, interface) == -1)
    return -1;

  wifi_->command_channel.nl80211_id = wifi_->notification_channel.nl80211_id;

  // subscribes channel to multicast group scan using scan group id
  if (mnl_socket_setsockopt(wifi_->notification_channel.nl, NETLINK_ADD_MEMBERSHIP,
                  &family_context.id_NL80211_MULTICAST_GROUP_SCAN, sizeof(int)) < 0) {
    ROS_ERROR("mnl_socket_set_sockopt error");
    return -1;
  }

  return 1;
}

// prerequisities:
// - proper interface, e.g. wlan0, wlan1
int Wifi::InitNetlinkChannel(struct netlink_channel *channel, const char *interface) {
  channel->sequence = 1;
  channel->buf = reinterpret_cast<char*>(malloc(MNL_SOCKET_BUFFER_SIZE));

  if (channel->buf == NULL) {
    ROS_ERROR("Insufficent memory for netlink socket buffer");
    return -1;
  }

  channel->ifindex = if_nametoindex(interface);

  if (channel->ifindex == 0) {
    ROS_ERROR("Incorrect network interface");
    return -1;
  }

  channel->context = NULL;

  if ((channel->nl = mnl_socket_open(NETLINK_GENERIC)) == NULL) {
    ROS_ERROR("mnl_socket_open error");
    return -1;
  }

  if (mnl_socket_bind(channel->nl, 0, MNL_SOCKET_AUTOPID) < 0) {
    ROS_ERROR("mnl_socket_bind error");
    return -1;
  }
  return 1;
}

// prerequisities:
// - channel initialized with init_netlink_channel
// - channel context of type context_CTRL_CMD_GETFAMILY
int Wifi::GetFamilyAndScanIds(struct netlink_channel *channel) {
  struct nlmsghdr *nlh = PrepareNlMessage(GENL_ID_CTRL, NLM_F_REQUEST | NLM_F_ACK,  CTRL_CMD_GETFAMILY, channel);
  mnl_attr_put_u16(nlh, CTRL_ATTR_FAMILY_ID, GENL_ID_CTRL);
  mnl_attr_put_strz(nlh, CTRL_ATTR_FAMILY_NAME, NL80211_GENL_NAME);

  SendNlMessage(nlh, channel);

  return ReceiveNlMessage(channel, handle_CTRL_CMD_GETFAMILY);
}

// prerequisities:
// - netlink_channel passed as data
// - data->context of type struct context_CTRL_CMD_GETFAMILY
int Wifi::handle_CTRL_CMD_GETFAMILY(const struct nlmsghdr *nlh, void *data) {
  struct nlattr *tb[CTRL_ATTR_MAX+1] = {};
  struct genlmsghdr *genl            = (struct genlmsghdr *)mnl_nlmsg_get_payload(nlh);
  struct netlink_channel *channel    = (struct netlink_channel*)data;
  struct validation_data vd          = {tb, CTRL_ATTR_MAX, NL80211_VALIDATION, NL80211_VALIDATION_LENGTH};

  mnl_attr_parse(nlh, sizeof(*genl), Validate, &vd);

  if (!tb[CTRL_ATTR_FAMILY_ID])
    ROS_ERROR("No family id attribute");

  channel->nl80211_id = mnl_attr_get_u16(tb[CTRL_ATTR_FAMILY_ID]);

  if (tb[CTRL_ATTR_MCAST_GROUPS])
    parse_CTRL_ATTR_MCAST_GROUPS(tb[CTRL_ATTR_MCAST_GROUPS], channel);

  return MNL_CB_OK;
}

// prerequisities:
// - data->context of type struct context_CTRL_CMD_GETFAMILY
void Wifi::parse_CTRL_ATTR_MCAST_GROUPS(struct nlattr *nested, struct netlink_channel *channel) {
  struct nlattr *pos;

  for ((pos) = (struct nlattr *) mnl_attr_get_payload(nested);
      mnl_attr_ok(pos, reinterpret_cast<char *>(mnl_attr_get_payload(nested))
        + mnl_attr_get_payload_len(nested) - reinterpret_cast<char *>(pos));
      (pos) = (struct nlattr *) mnl_attr_next(pos)) {
    struct nlattr *tb[CTRL_ATTR_MCAST_GRP_MAX+1] = {};
    struct validation_data vd={tb, CTRL_ATTR_MCAST_GRP_MAX,
                               NL80211_MCAST_GROUPS_VALIDATION, NL80211_MCAST_GROUPS_VALIDATION_LENGTH};

    mnl_attr_parse_nested(pos, Validate, &vd);

    if (tb[CTRL_ATTR_MCAST_GRP_NAME]) {
     const char *name = mnl_attr_get_str(tb[CTRL_ATTR_MCAST_GRP_NAME]);

    if (strcmp(name, "scan") == 0) {
      if (tb[CTRL_ATTR_MCAST_GRP_ID]) {
        struct context_CTRL_CMD_GETFAMILY *context = (struct context_CTRL_CMD_GETFAMILY *) channel->context;
        context->id_NL80211_MULTICAST_GROUP_SCAN = mnl_attr_get_u32(tb[CTRL_ATTR_MCAST_GRP_ID]);
      } else {
        ROS_ERROR("Missing id attribute for scan multicast group");
      }
    }
  }
}
}

// CLEANUP

// prerequisities:
// - wifi_ initialized with wifi_scan_init
void Wifi::WifiScanClose() {
  CloseNetlinkChannel(&wifi_->notification_channel);
  CloseNetlinkChannel(&wifi_->command_channel);
  free(wifi_);
}

// prerequisities:
// - channel initalized with init_netlink-channel
void Wifi::CloseNetlinkChannel(struct netlink_channel *channel) {
  free(channel->buf);
  mnl_socket_close(channel->nl);
}

// SCANNING

// handle also trigger abort
// public interface
//
// prerequisities:
// - wifi_ initialized with wifi_scan_init
// - bss_info table of sized bss_info_length passed
int Wifi::WifiScanAll(struct bss_info *bss_infos, int bss_infos_length) {
  struct netlink_channel *notifications = &wifi_->notification_channel;
  struct context_NL80211_MULTICAST_GROUP_SCAN scanning = {0, 0};
  notifications->context = &scanning;

  struct netlink_channel *commands=&wifi_->command_channel;
  struct context_NL80211_CMD_NEW_SCAN_RESULTS scan_results = {bss_infos, bss_infos_length, 0};
  commands->context = &scan_results;

  // somebody else might have triggered scanning or even the results can be already waiting
  if (ReadPastNotifications(notifications) == -1)
    return -1;

  // if no results yet or scan not triggered then trigger it.
  // the device can be busy - we have to take it into account
  if (TriggerScanIfNecessary(commands, &scanning) == -1)
    return -1;  // most likely with errno set to EBUSY

  // now just wait for trigger/new_scan_results
  if (WaitForNewScanResults(notifications) == -1)
    return -1;

  // finally read the scan
  GetScan(commands);

  return scan_results.scanned;
}

// SCANNING - notification related

// prerequisities
// - subscribed to scan group with subscribe_NL80211_MULTICAST_GROUP_SCAN
// - context_NL80211_MULTICAST_GROUP_SCAN set for notifications
int Wifi::ReadPastNotifications(struct netlink_channel *notifications) {
  int fd, flags, ret, run_ret;

  // Set channer non blocking
  fd = mnl_socket_get_fd(notifications->nl);
  flags = fcntl(fd, F_GETFL, 0);
  if (flags == -1) {
    ROS_ERROR("SetChannelNonBlocking F_GETFL");
    return -1;
  }
  if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
    ROS_ERROR("SetChannelNonBlocking F_SETFL");
    return -1;
  }

  while ((ret = mnl_socket_recvfrom(notifications->nl, notifications->buf, MNL_SOCKET_BUFFER_SIZE) ) >= 0) {
    // the line below fills context about past scans/triggers
    run_ret = mnl_cb_run(notifications->buf, ret, 0, 0, handle_NL80211_MULTICAST_GROUP_SCAN, notifications);
    if (run_ret <= 0) {
      ROS_ERROR("ReadPastNotificationsNonBlocking mnl_cb_run failed");
    return -1;
    }
  }

  if (ret == -1) {
    if (!(errno == EINPROGRESS || errno == EWOULDBLOCK)) {
      ROS_ERROR("ReadPastNotificationsNonBlocking mnl_socket_recv failed");
      return -1;
    }
  }
  // no more notifications waiting

  // Set channel blocking
  fd = mnl_socket_get_fd(notifications->nl);
  flags = fcntl(fd, F_GETFL, 0);
  if (flags == -1) {
    ROS_ERROR("SetChannelNonBlocking F_GETFL");
    return -1;
  }
  if (fcntl(fd, F_SETFL, flags &  ~O_NONBLOCK) == -1) {
    ROS_ERROR("SetChannelNonBlocking F_SETFL");
    return -1;
  }
  return 1;
}

// prerequisities:
// - subscribed to scan group with subscribe_NL80211_MULTICAST_GROUP_SCAN
// - netlink_channel passed as data
// - data->context of type struct context_NL80211_MULTICAST_GROUP_SCAN
int Wifi::handle_NL80211_MULTICAST_GROUP_SCAN(const struct nlmsghdr *nlh, void *data) {
  struct netlink_channel *channel = (struct netlink_channel *) data;
  struct context_NL80211_MULTICAST_GROUP_SCAN *context =
              (struct context_NL80211_MULTICAST_GROUP_SCAN *) channel->context;
  struct genlmsghdr *genl = (struct genlmsghdr *)mnl_nlmsg_get_payload(nlh);

  if (genl->cmd == NL80211_CMD_TRIGGER_SCAN) {
    context->scan_triggered = 1;
    return MNL_CB_OK;  // do nothing for now
  } else if (genl->cmd == NL80211_CMD_NEW_SCAN_RESULTS) {
    // new scan results
    if (nlh->nlmsg_pid == 0 &&  nlh->nlmsg_seq == 0)
      context->new_scan_results = 1;
    return MNL_CB_OK;  // do nothing for now
  } else {
    ROS_DEBUG("Ignoring generic netlink command type %u seq %u pid  %u genl cmd %u\n",
                         nlh->nlmsg_type, nlh->nlmsg_seq, nlh->nlmsg_pid, genl->cmd);
    return MNL_CB_OK;
  }
}

// prerequisities:
// - commands initialized with init_netlink_channel
// - scanning updated with ReadPastNotifications
int Wifi::TriggerScanIfNecessary(struct netlink_channel *commands,
                                     struct context_NL80211_MULTICAST_GROUP_SCAN *scanning) {
  if (!scanning->new_scan_results && !scanning->scan_triggered) {
    struct nlmsghdr *nlh = PrepareNlMessage(commands->nl80211_id, NLM_F_REQUEST
                                          | NLM_F_ACK, NL80211_CMD_TRIGGER_SCAN, commands);

    mnl_attr_put_u32(nlh,  NL80211_ATTR_IFINDEX, commands->ifindex);
    SendNlMessage(nlh, commands);

    if (ReceiveNlMessage(commands, handle_NL80211_CMD_NEW_SCAN_RESULTS) == -1)
      return -1;  // most likely errno set to EBUSY which means hardware is doing something else, try again later
  }
  return 0;
}

// prerequisities
// - channel initalized with init_netlink_channel
// - subscribed to scan group with subscribe_NL80211_MULTICAST_GROUP_SCAN
// - context_NL80211_MULTICAST_GROUP_SCAN set for notifications
int Wifi::WaitForNewScanResults(struct netlink_channel *notifications) {
  struct context_NL80211_MULTICAST_GROUP_SCAN *scanning =
                              (struct context_NL80211_MULTICAST_GROUP_SCAN *) notifications->context;
  int ret;
  while (!scanning->new_scan_results) {
    if ((ret = mnl_socket_recvfrom(notifications->nl, notifications->buf, MNL_SOCKET_BUFFER_SIZE)) <= 0) {
      ROS_ERROR("Waiting for new scan results failed - mnl_socket_recvfrom");
      return -1;
    }

    if ((ret = mnl_cb_run(notifications->buf, ret, 0, 0, handle_NL80211_MULTICAST_GROUP_SCAN, notifications)) <=0) {
      ROS_ERROR("Processing notificatoins failed - mnl_cb_run");
      return -1;
    }
  }
  return 1;
}

// SCANNING - scan related

// prerequisities:
// - channel initalized with init_netlink_channel
// - channel context of type context_NL80211_CMD_NEW_SCAN_RESULTS
int Wifi::GetScan(struct netlink_channel *channel) {
  struct nlmsghdr *nlh = PrepareNlMessage(channel->nl80211_id, NLM_F_REQUEST
                                          | NLM_F_DUMP | NLM_F_ACK, NL80211_CMD_GET_SCAN, channel);
  mnl_attr_put_u32(nlh,  NL80211_ATTR_IFINDEX, channel->ifindex);

  SendNlMessage(nlh, channel);
  return ReceiveNlMessage(channel, handle_NL80211_CMD_NEW_SCAN_RESULTS);
}

// prerequisities:
// - netlink_channel passed as data
// - data->context of type context_NL80211_CMD_NEW_SCAN_RESULTS
int Wifi::handle_NL80211_CMD_NEW_SCAN_RESULTS(const struct nlmsghdr *nlh, void *data) {
  struct netlink_channel *channel = (struct netlink_channel *) data;
  struct nlattr *tb[NL80211_ATTR_MAX+1] = {};
  struct validation_data vd = {tb, NL80211_ATTR_MAX, NL80211_NEW_SCAN_RESULTS_VALIDATION,
                               NL80211_NEW_SCAN_RESULTS_VALIDATION_LENGTH};
  struct genlmsghdr *genl = (struct genlmsghdr *)mnl_nlmsg_get_payload(nlh);

  if (genl->cmd != NL80211_CMD_NEW_SCAN_RESULTS) {
    ROS_DEBUG("Ignoring generic netlink command %u seq %u pid  %u genl cmd %u\n",
                       nlh->nlmsg_type, nlh->nlmsg_seq, nlh->nlmsg_pid, genl->cmd);
    return MNL_CB_OK;
  }

  mnl_attr_parse(nlh, sizeof(*genl), Validate, &vd);

  if (!tb[NL80211_ATTR_BSS])
    return MNL_CB_OK;

  parse_NL80211_ATTR_BSS(tb[NL80211_ATTR_BSS], channel);

  return MNL_CB_OK;
}

// prerequisities:
// - channel context of type context_NL80211_CMD_NEW_SCAN_RESULTS
void Wifi::parse_NL80211_ATTR_BSS(struct nlattr *nested, struct netlink_channel *channel) {
  struct nlattr *tb[NL80211_BSS_MAX+1] = {};
  struct validation_data vd = {tb, NL80211_BSS_MAX, NL80211_BSS_VALIDATION,
                               NL80211_BSS_VALIDATION_LENGTH};
  struct context_NL80211_CMD_NEW_SCAN_RESULTS *scan_results =
                        (struct context_NL80211_CMD_NEW_SCAN_RESULTS *) channel->context;
  struct bss_info *bss = scan_results->bss_infos + scan_results->scanned;

  mnl_attr_parse_nested(nested, Validate, &vd);

  enum nl80211_bss_status status = (enum nl80211_bss_status) BSS_NONE;

  if (tb[NL80211_BSS_STATUS])
    status = (enum nl80211_bss_status) mnl_attr_get_u32(tb[NL80211_BSS_STATUS]);

  // if we have found associated station store first as last and associated as first
  if (status == NL80211_BSS_STATUS_ASSOCIATED || status == NL80211_BSS_STATUS_ASSOCIATED
      || status == NL80211_BSS_STATUS_IBSS_JOINED) {
    if (scan_results->scanned > 0 && scan_results->scanned < scan_results->bss_infos_length)
      memcpy(bss, scan_results->bss_infos, sizeof(struct bss_info));
    bss = scan_results->bss_infos;
  }

  // check bounds, make exception if we have found associated station and replace previous data
  if (scan_results->bss_infos_length == 0 || (scan_results->scanned >= scan_results->bss_infos_length
      && bss != scan_results->bss_infos)) {
    ++scan_results->scanned;
    return;
  }

  if (tb[NL80211_BSS_BSSID])
    parse_NL80211_BSS_BSSID(tb[NL80211_BSS_BSSID], bss->bssid);

  if (tb[NL80211_BSS_FREQUENCY])
    bss->frequency = mnl_attr_get_u32(tb[NL80211_BSS_FREQUENCY]);

  if (tb[NL80211_BSS_INFORMATION_ELEMENTS])
    parse_NL80211_BSS_INFORMATION_ELEMENTS(tb[NL80211_BSS_INFORMATION_ELEMENTS], bss->ssid);

  if (tb[NL80211_BSS_SIGNAL_MBM])
    bss->signal_mbm = mnl_attr_get_u32(tb[NL80211_BSS_SIGNAL_MBM]);

  if (tb[NL80211_BSS_SEEN_MS_AGO])
    bss->seen_ms_ago = mnl_attr_get_u32(tb[NL80211_BSS_SEEN_MS_AGO]);

  bss->status = (bss_status) status;

  ++scan_results->scanned;
}

// This is guesswork! Read up on that!!! I don't think it's netlink in this attribute, some lower beacon layer
void Wifi::parse_NL80211_BSS_INFORMATION_ELEMENTS(struct nlattr *attr, char SSID_OUT[SSID_MAX_LENGTH_WITH_NULL]) {
  const char *payload = reinterpret_cast<char *>(mnl_attr_get_payload(attr));
  int len = mnl_attr_get_payload_len(attr);
  if (len == 0 || payload[0] != 0 || payload[1] >= SSID_MAX_LENGTH_WITH_NULL || payload[1] > len-2) {
    ROS_DEBUG("SSID len 0 or payload not starting from 0 or payload length > 32 or payload length > length-2!\n");
    SSID_OUT[0] = '\0';
    return;
  }
  int ssid_len = payload[1];
  strncpy(SSID_OUT, payload+2, ssid_len);
  SSID_OUT[ssid_len] = '\0';
}

void Wifi::parse_NL80211_BSS_BSSID(struct nlattr *attr, uint8_t bssid_out[BSSID_LENGTH]) {
  const char *payload = reinterpret_cast<char *>(mnl_attr_get_payload(attr));
  int len = mnl_attr_get_payload_len(attr);

  if (len != BSSID_LENGTH) {
    ROS_DEBUG("BSSID length != %d, ignoring", BSSID_LENGTH);
    memset(bssid_out, 0, BSSID_LENGTH);
    return;
  }

  memcpy(bssid_out, payload, BSSID_LENGTH);
}

// STATION

// public interface
//
// prerequisities:
int Wifi::WifiScanStation(struct station_info *station) {
  struct netlink_channel *commands = &wifi_->command_channel;
  struct bss_info bss;

  struct context_NL80211_CMD_NEW_SCAN_RESULTS scan_results = {&bss, 1, 0};
  commands->context = &scan_results;
  GetScan(commands);

  if (scan_results.scanned == 0)
    return 0;

  struct context_NL80211_CMD_NEW_STATION station_results = {station};
  commands->context = &station_results;
  GetStation(commands, bss.bssid);

  memcpy(station->bssid, bss.bssid, BSSID_LENGTH);
  memcpy(station->ssid, bss.ssid, SSID_MAX_LENGTH_WITH_NULL);
  station->status = bss.status;

  return 1;
}

// prerequisites:
// - channel initalized with init_netlink_channel
// - context_NL80211_CMD_NEW_STATION set for channel
int Wifi::GetStation(struct netlink_channel *channel, uint8_t bssid[BSSID_LENGTH]) {
  struct nlmsghdr *nlh = PrepareNlMessage(channel->nl80211_id, NLM_F_REQUEST | NLM_F_ACK,
                                          NL80211_CMD_GET_STATION, channel);
  mnl_attr_put_u32(nlh,  NL80211_ATTR_IFINDEX, channel->ifindex);
  mnl_attr_put(nlh,  NL80211_ATTR_MAC, BSSID_LENGTH, bssid);
  SendNlMessage(nlh, channel);
  return ReceiveNlMessage(channel, handle_NL80211_CMD_NEW_STATION);
}

// prerequisities:
// - netlink_channel passed as data
// - data->context of type context_NL80211_CMD_NEW_STATION
int Wifi::handle_NL80211_CMD_NEW_STATION(const struct nlmsghdr *nlh, void *data) {
  struct netlink_channel *channel = (struct netlink_channel *) data;
  struct nlattr *tb[NL80211_ATTR_MAX+1] = {};
  struct validation_data vd = {tb, NL80211_ATTR_MAX,
                               NL80211_CMD_NEW_STATION_VALIDATION, NL80211_CMD_NEW_STATION_VALIDATION_LENGTH};
  struct genlmsghdr *genl = (struct genlmsghdr *)mnl_nlmsg_get_payload(nlh);

  if (genl->cmd != NL80211_CMD_NEW_STATION) {
    ROS_DEBUG("Ignoring generic netlink command %u seq %u pid  %u genl cmd %u\n",
                                     nlh->nlmsg_type, nlh->nlmsg_seq, nlh->nlmsg_pid, genl->cmd);
    return MNL_CB_OK;
  }

  mnl_attr_parse(nlh, sizeof(*genl), Validate, &vd);

  if (!tb[NL80211_ATTR_STA_INFO])  // or error, no statoin
    return MNL_CB_OK;

  parse_NL80211_ATTR_STA_INFO(tb[NL80211_ATTR_STA_INFO], channel);

  return MNL_CB_OK;
}

// prerequisities:
// - channel context of type context_NL80211_CMD_NEW_STATION
void Wifi::parse_NL80211_ATTR_STA_INFO(struct nlattr *nested, struct netlink_channel *channel) {
  struct nlattr *tb[NL80211_STA_INFO_MAX+1] = {};
  struct validation_data vd = {tb, NL80211_STA_INFO_MAX,
                               NL80211_STA_INFO_VALIDATION, NL80211_STA_INFO_VALIDATION_LENGTH};
  struct context_NL80211_CMD_NEW_STATION *station_results = (struct context_NL80211_CMD_NEW_STATION *) channel->context;
  struct station_info *station = station_results->station;

  mnl_attr_parse_nested(nested, Validate, &vd);

  if ( tb[NL80211_STA_INFO_SIGNAL])
    station->signal_dbm     = (int8_t)mnl_attr_get_u8(tb[NL80211_STA_INFO_SIGNAL]);
  if ( tb[NL80211_STA_INFO_SIGNAL_AVG])
    station->signal_avg_dbm = (int8_t)mnl_attr_get_u8(tb[NL80211_STA_INFO_SIGNAL_AVG]);
  if (tb[NL80211_STA_INFO_RX_PACKETS])
    station->rx_packets    = mnl_attr_get_u32(tb[NL80211_STA_INFO_RX_PACKETS]);
  if (tb[NL80211_STA_INFO_TX_PACKETS])
    station->tx_packets    = mnl_attr_get_u32(tb[NL80211_STA_INFO_TX_PACKETS]);
}

// NETLINK HELPERS

// NETLINK HELPERS - message construction/sending/receiving

// prerequisities:
// - channel initialized with init_netlink_channel
struct nlmsghdr *Wifi::PrepareNlMessage(uint32_t type, uint16_t flags,
                                        uint8_t genl_cmd, struct netlink_channel *channel) {
  struct nlmsghdr *nlh;
  struct genlmsghdr *genl;

  nlh = mnl_nlmsg_put_header(channel->buf);
  nlh->nlmsg_type = type;
  nlh->nlmsg_flags = flags;
  nlh->nlmsg_seq = channel->sequence;

  genl = (struct genlmsghdr*)mnl_nlmsg_put_extra_header(nlh, sizeof(struct genlmsghdr));
  genl->cmd = genl_cmd;
  genl->version = 1;
  return nlh;
}

// prerequisities:
// - PrepareNlMessage called first
// - mnl_attr_put_xxx used if additional attributes needed
void Wifi::SendNlMessage(struct nlmsghdr *nlh, struct netlink_channel *channel) {
  if (mnl_socket_sendto(channel->nl, nlh, nlh->nlmsg_len) < 0)
    ROS_ERROR("mnl_socket_sendto");
}

// prerequisities:
// - SendNlMessage called first
// - prerequisities for callback matched
int Wifi::ReceiveNlMessage(struct netlink_channel *channel, mnl_cb_t callback) {
  int ret;
  unsigned int portid = mnl_socket_get_portid(channel->nl);

  ret = mnl_socket_recvfrom(channel->nl, channel->buf, MNL_SOCKET_BUFFER_SIZE);

  while (ret > 0) {
    ret = mnl_cb_run(channel->buf, ret, channel->sequence, portid, callback, channel);
    if (ret <= 0)
      break;
    ret = mnl_socket_recvfrom(channel->nl, channel->buf, MNL_SOCKET_BUFFER_SIZE);
  }

  ++channel->sequence;

  return ret;
}

// NETLINK HELPERS - validation

// prerequisities:
// - data of type validation_data
int Wifi::Validate(const struct nlattr *attr, void *data) {
  struct validation_data *vd = (struct validation_data *) data;
  const struct nlattr **tb = (const struct nlattr**) vd->attribute_table;
  int type = mnl_attr_get_type(attr), i;

  if (mnl_attr_type_valid(attr, vd->attribute_length) < 0)
    return MNL_CB_OK;

  for (i = 0; i < vd->validation_length; ++i)
    if (type == vd->validation[i].attr) {
      int len = vd->validation[i].len;
      if (len == 0 && mnl_attr_validate(attr, vd->validation[i].type) < 0) {
       perror("mnl_attr_validate error");
       return MNL_CB_ERROR;
     }
     if (len != 0 && mnl_attr_validate2(attr, vd->validation[i].type, len) < 0) {
      perror("mnl_attr_validate error");
      return MNL_CB_ERROR;
    }
  }

  tb[type] = attr;
  return MNL_CB_OK;
}
}  // namespace wifi
