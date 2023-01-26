############################################################################
# apps/examples/hello/Make.defs
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(APPDIR)/Make.defs

# Set up build configuration and environment

WD := ${shell echo $(CURDIR) | sed -e 's/ /\\ /g'}

CANOPENNODE_SRCDIR = $(WD)/CANOpenGit
CANOPENNODE_CANOPENNODEDIR = $(CANOPENNODE_SRCDIR)/CANopenNode

# CANOpenNode Linux

CSRCS += $(CANOPENNODE_SRCDIR)/CO_driver.c
CSRCS += $(CANOPENNODE_SRCDIR)/CO_epoll_interface.c
CSRCS += $(CANOPENNODE_SRCDIR)/CO_error.c
CSRCS += $(CANOPENNODE_SRCDIR)/CO_storageLinux.c
CFLAGS += -I$(CANOPENNODE_SRCDIR)

# CANOpenNode testProfile

CSRCS += $(WD)/testProfile/OD.c
CFLAGS += -I$(WD)/testProfile

# CANOpenNode

CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/CANopen.c
CFLAGS += -I$(CANOPENNODE_CANOPENNODEDIR)

# 301

CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/301/CO_Emergency.c
CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/301/CO_fifo.c
CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/301/CO_HBconsumer.c
CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/301/CO_NMT_Heartbeat.c
CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/301/CO_ODinterface.c
CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/301/CO_PDO.c
CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/301/CO_SDOclient.c
CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/301/CO_SDOserver.c
CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/301/CO_SYNC.c
CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/301/CO_TIME.c
CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/301/crc16-ccitt.c

# 303

CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/303/CO_LEDs.c

# 304

CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/304/CO_GFC.c
CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/304/CO_SRDO.c

# 305

CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/305/CO_LSSmaster.c
CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/305/CO_LSSslave.c

# 309

CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/309/CO_gateway_ascii.c

# storage

CSRCS += $(CANOPENNODE_CANOPENNODEDIR)/storage/CO_storage.c


PROGNAME  = conode
PRIORITY  = $(CONFIG_CAN_OPEN_NODE_PRIORITY)
STACKSIZE = $(CONFIG_CAN_OPEN_NODE_STACKSIZE)
MODULE    = $(CONFIG_CAN_OPEN_NODE)

# Hello, World! Example

MAINSRC = CANOpenGit/CO_main_basic.c



include $(APPDIR)/Application.mk
