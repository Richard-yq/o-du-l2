################################################################################
#   Copyright (c)  		                            			               #
####################### [Small Cell Integration] ###############################
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
# 									   										   #
#                                                                              #
#					Author Brandon.Chuang / Yu-Hsin Chuang					   #
#                                                                              #
#                                                                              #
################################################################################

# This is makefile for open-nFAPI module
include ../common/rsys_fancy.mak
include ../common/env.mak

COLOR=$(COLOR_RED)

SRC_DIR=$(ROOT_DIR)/src/nfapi/open-nFAPI/pnf/src
C_SRCS=$(wildcard $(SRC_DIR)/*.c)
C_OBJS=$(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(C_SRCS))

I_OPTS+=-I$(ROOT_DIR)/src/nfapi/open-nFAPI/common/public_inc
I_OPTS+=-I$(ROOT_DIR)/src/nfapi/open-nFAPI/nfapi/public_inc
I_OPTS+=-I$(ROOT_DIR)/src/nfapi/open-nFAPI/pnf/public_inc
I_OPTS+=-I$(ROOT_DIR)/src/nfapi/open-nFAPI/pnf/inc

# # prepare the list of the odu header files
HDR_FILES+=$(wildcard $(SRC_DIR)/*.[hx])

lib: $(LIB_DIR)/libnfapi_pnf.a
include $(COM_BUILD_DIR)/compile.mak

#-------------------------------------------------------------#
#Linker macros
#-------------------------------------------------------------#
$(LIB_DIR)/libnfapi_pnf.a:$(C_OBJS)
		  @echo -e "Creating Archive $(COLOR) $@ $(REVERT_COLOR)"
		  $(Q)ar -cr $(LIB_DIR)/libnfapi_pnf.a $(C_OBJS)

#-------------------------------------------------------------#
#Clean macros
#-------------------------------------------------------------#
clean:
		  @echo -e "$(COLOR_RED)Cleaning NFAPI_PNF$(REVERT_COLOR)"
		  $(Q)\rm -f $(LIB_DIR)/libnfapi_pnf.a $(C_OBJS) 

#**********************************************************************
#         End of file
#**********************************************************************