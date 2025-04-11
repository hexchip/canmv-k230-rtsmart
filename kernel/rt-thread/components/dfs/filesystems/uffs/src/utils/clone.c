/*
  This file is part of UFFS, the Ultra-low-cost Flash File System.

  Copyright (C) 2005-2009 Ricky Zheng <ricky_gz_zheng@yahoo.co.nz>

  UFFS is free software; you can redistribute it and/or modify it under
  the GNU Library General Public License as published by the Free Software
  Foundation; either version 2 of the License, or (at your option) any
  later version.

  UFFS is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
  or GNU Library General Public License, as applicable, for more details.

  You should have received a copy of the GNU General Public License
  and GNU Library General Public License along with UFFS; if not, write
  to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
  Boston, MA  02110-1301, USA.

  As a special exception, if other files instantiate templates or use
  macros or inline functions from this file, or you compile this file
  and link it with other works to produce a work based on this file,
  this file does not by itself cause the resulting work to be covered
  by the GNU General Public License. However the source code for this
  file must still be made available in accordance with section (3) of
  the GNU General Public License v2.

  This exception does not invalidate any other reasons why a work based
  on this file might be covered by the GNU General Public License.
*/

#include "uffs/uffs_core.h"
#include "uffs/uffs_fd.h"
#include "uffs/uffs_find.h"
#include "uffs/uffs_fs.h"
#include "uffs/uffs_mtb.h"
#include "uffs/uffs_public.h"
#include "uffs/uffs_utils.h"
#include "uffs_config.h"

#include <dirent.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define PFX "clone: "
#define MSG(msg, ...) uffs_PerrorRaw(UFFS_MSG_NORMAL, msg, ##__VA_ARGS__)
#define MSGLN(msg, ...) uffs_Perror(UFFS_MSG_NORMAL, msg, ##__VA_ARGS__)

static char buf[4096];

/**
 * @brief 递归复制主机目录到 UFFS（从 / 开始）
 * @param host_path 主机文件/目录路径
 * @param uffs_base_path UFFS 基础路径（固定为 /）
 * @param relative_path 相对路径（用于递归）
 * @return 0 成功，-1 失败
 */
static int clone_to_uffs_recursive(const char *host_path,
                                   const char *uffs_base_path,
                                   const char *relative_path) {
  struct stat st;
  if (stat(host_path, &st) != 0) {
    MSGLN("Error: Cannot access %s", host_path);
    return -1;
  }

  char uffs_full_path[PATH_MAX];
  if (relative_path[0] == '\0') {
    snprintf(uffs_full_path, sizeof(uffs_full_path), "%s", uffs_base_path);
  } else {
    snprintf(uffs_full_path, sizeof(uffs_full_path), "%s%s", uffs_base_path,
             relative_path);
  }

  if (S_ISDIR(st.st_mode)) {
    if(0x01 != strlen(uffs_full_path)) {
        if (uffs_mkdir(uffs_full_path) < 0) {
            MSGLN("Error: Failed to create UFFS directory %s (err: %d)",
                  uffs_full_path, uffs_get_error());
            return -1;
          }
    }

    DIR *dir = opendir(host_path);
    if (!dir) {
      MSGLN("Error: Cannot open host directory %s", host_path);
      return -1;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
      if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
        continue;
      }

      char host_child_path[PATH_MAX];
      char child_relative_path[PATH_MAX];
      snprintf(host_child_path, sizeof(host_child_path), "%s/%s", host_path,
               entry->d_name);

      if (relative_path[0] == '\0') {
        snprintf(child_relative_path, sizeof(child_relative_path), "%s",
                 entry->d_name);
      } else {
        snprintf(child_relative_path, sizeof(child_relative_path), "%s/%s",
                 relative_path, entry->d_name);
      }

      if (clone_to_uffs_recursive(host_child_path, uffs_base_path,
                                  child_relative_path) < 0) {
        closedir(dir);
        return -1;
      }
    }
    closedir(dir);
  } else if (S_ISREG(st.st_mode)) {
    // 复制文件内容
    FILE *host_file = fopen(host_path, "rb");
    if (!host_file) {
      MSGLN("Error: Cannot open host file %s", host_path);
      return -1;
    }

    int uffs_fd = uffs_open(uffs_full_path, UO_RDWR | UO_CREATE | UO_TRUNC);
    if (uffs_fd < 0) {
      MSGLN("Error: Cannot create UFFS file %s (err: %d)", uffs_full_path,
            uffs_get_error());
      fclose(host_file);
      return -1;
    }

    size_t bytes_read;
    while ((bytes_read = fread(buf, 1, sizeof(buf), host_file)) > 0) {
      if (uffs_write(uffs_fd, buf, bytes_read) != (int)bytes_read) {
        MSGLN("Error: Failed to write to UFFS file %s (err: %d)",
              uffs_full_path, uffs_get_error());
        fclose(host_file);
        uffs_close(uffs_fd);
        return -1;
      }
    }

    fclose(host_file);
    uffs_close(uffs_fd);
    MSGLN("Copied: %s -> %s", host_path, uffs_full_path);
  } else {
    MSGLN("Error: %s is not a regular file or directory", host_path);
    return -1;
  }

  return 0;
}

/**
 * @brief 将主机目录复制到 UFFS 根目录 /
 * @param host_path 主机路径（如 /home/test/xxx）
 * @return 0 成功，-1 失败
 */
int clone_uffs(const char *host_path) {
  char *path = strdup(host_path);
  if (!path) {
    MSGLN("Error: Memory allocation failed");
    return -1;
  }

  size_t len = strlen(path);
  if (len > 0 && path[len - 1] == '/') {
    path[len - 1] = '\0';
  }

  // 递归复制，UFFS 目标路径固定为 /，相对路径为空
  int ret = clone_to_uffs_recursive(path, "/", "");
  free(path);

  return ret;
}
