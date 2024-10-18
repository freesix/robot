#pragma once

extern const char imu_cfg_cmd[50][50];

int imu_cfg_cmd_send(const char *string, unsigned int slen);