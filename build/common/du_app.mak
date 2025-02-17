################################################################################
#   Copyright (c) [2017-2019] [Radisys]                                        #
#                                                                              #
#   Licensed under the Apache License, Version 2.0 (the "License");            #
#   you may not use this file except in compliance with the License.           #
#   You may obtain a copy of the License at                                    #
#                                                                              #
#       http://www.apache.org/licenses/LICENSE-2.0                             #
#                                                                              #
#   Unless required by applicable law or agreed to in writing, software        #
#   distributed under the License is distributed on an "AS IS" BASIS,          #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
#   See the License for the specific language governing permissions and        #
#   limitations under the License.                                             #
################################################################################

# This is makefile for DU APP

include ../common/rsys_fancy.mak
include ../common/env.mak
COLOR=$(COLOR_RED)

SRC_DIR=$(ROOT_DIR)/src/du_app/
C_SRCS=$(wildcard $(SRC_DIR)/*.c)
C_OBJS=$(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(C_SRCS))

# prepare the list of common header files
HDR_FILES+=$(wildcard $(CM_DIR)/env*.[hx])
HDR_FILES+=$(wildcard $(CM_DIR)/gen*.[hx])
HDR_FILES+=$(wildcard $(CM_DIR)/ssi*.[hx])
HDR_FILES+=$(wildcard $(CM_DIR)/cm*.[hx])
HDR_FILES+=$(wildcard $(CM_DIR)/lkw*.[hx])
HDR_FILES+=$(wildcard $(CM_DIR)/lrg*.[hx])

lib: $(LIB_DIR)/libduapp.a
include $(COM_BUILD_DIR)/compile.mak

ifdef XML_BASED_CONFIG
I_OPTS+=-I/usr/include/libxml2
endif
I_OPTS+=-I$(ROOT_DIR)/src/mt
I_OPTS+=-I$(ROOT_DIR)/src/codec_utils/common
I_OPTS+=-I$(ROOT_DIR)/src/codec_utils/F1AP
I_OPTS+=-I$(ROOT_DIR)/src/codec_utils/RRC
I_OPTS+=-I$(ROOT_DIR)/src/codec_utils/E2AP
I_OPTS+=-I$(ROOT_DIR)/src/codec_utils/E2SM_KPM

ifeq ($(NFAPI),YES)
I_OPTS+=-I$(ROOT_DIR)/src/nfapi/open-nFAPI/vnf/public_inc
I_OPTS+=-I$(ROOT_DIR)/src/nfapi/open-nFAPI/nfapi/public_inc
I_OPTS+=-I$(ROOT_DIR)/src/nfapi/open-nFAPI/common/public_inc
I_OPTS+=-I$(ROOT_DIR)/src/nfapi/oai_integration
I_OPTS+=-I$(ROOT_DIR)/src/5gnrmac
endif

ifeq ($(O1_ENABLE),YES)
I_OPTS+=-I$(ROOT_DIR)/src/o1
endif

#-------------------------------------------------------------#
#Linker macros
#-------------------------------------------------------------#
$(LIB_DIR)/libduapp.a:$(C_OBJS)
		  @echo -e "Creating Archive $(COLOR) $@ $(REVERT_COLOR)"
		  $(Q)ar -cr $(LIB_DIR)/libduapp.a $(C_OBJS) 

#-------------------------------------------------------------#
#Clean macros
#-------------------------------------------------------------#
clean:
		  @echo -e "$(COLOR_RED)Cleaning DU APP$(REVERT_COLOR)"
		  @echo $(SRC_DIR) $(CM_DIR)
		  $(Q)\rm -f $(LIB_DIR)/libduapp.a $(C_OBJS) 

#**********************************************************************
#         End of file
#**********************************************************************
