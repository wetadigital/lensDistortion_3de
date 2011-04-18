
########################################################################
#
# Copyright (c) 2011, Weta Digital Limited
#
# All rights reserved.
#
#    Redistribution and use in source and binary forms, with or 
#    without modification, are permitted provided that the 
#    following conditions are met:
#
#     * Redistributions of source code must retain the above 
#       copyright notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above 
#       copyright notice, this list of conditions and the following disclaimer 
#       in the documentation and/or other materials provided with the distribution.
#
#     * Neither the name of the Weta Digital nor the names of its contributors 
#       may be used to endorse or promote products derived from this 
#       software without specific prior written permission.
#
#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
#    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
#    COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
#    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
#    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
#    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
#    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
#    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
#    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
#    POSSIBILITY OF SUCH DAMAGE.
#
#
####################################################################

#
# Locate your ldpk and adjust accordingly!
#

LDPK_PATH	:=/usr/local/ldpk/
LDPK_INCPATH	:=${LDPK_PATH}/include
LDPK_LIB	:=-L ${LDPK_PATH}/lib -lldpk
LENSMODEL_PATH	:=/usr/local/lensmodels/


###
#  automatically looks for a version of nuke, build against that
#  if all else fails, replace with a straight path
####

NUKE_EXE 	:=$(shell which Nuke6.3)

ifeq (">$(NUKE_EXE)<","><")
NUKE_EXE :=$(shell which Nuke6.2)
endif
ifeq (">$(NUKE_EXE)<","><")
NUKE_EXE :=$(shell which Nuke6.1)
endif
ifeq (">$(NUKE_EXE)<","><")
NUKE_EXE :=$(shell which Nuke6.0)
endif
ifeq (">$(NUKE_EXE)<","><")
$(error cannot find any version of nuke. ensure it is in a path somewhere)
endif

NUKE_DIR 	:=$(shell dirname ${NUKE_EXE})
NUKE_INC 	:= -I $(NUKE_DIR)/include

lensDistortion_3de.so: lensDistortion_3de.cc ${LENSMODEL_PATH} ${LDPK_INCPATH}/ldpk/ldpk_plugin_loader.h
	$(CC) -fPIC -shared -o lensDistortion_3de.so lensDistortion_3de.cc -DLENSMODEL_PATH=\"${LENSMODEL_PATH}\" $(NUKE_INC) -I $(LDPK_INCPATH) $(LDPK_LIB)

${LENSMODEL_PATH}:
	$(error please specify path to ldpk plugins in LENSMODEL_PATH - will be used as a fallback if LENSMODEL_PATH not set as an environment variable)

${LDPK_INCPATH}/ldpk/ldpk_plugin_loader.h:
	$(error cant locate an installation of ldpk - obtain from 3DEqualizer.com, install, and update the Makefile with the correct path)
clean:
	rm lensDistortion_3de.so
