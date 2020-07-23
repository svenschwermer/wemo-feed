#
# Copyright (C) 2020 Sven Schwermer
#
# This is free software, licensed under the GNU General Public License v2.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=wemo-api
PKG_VERSION:=0.1
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0

include $(INCLUDE_DIR)/package.mk

define Package/wemo-api
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Control the relay via HTTP
  DEPENDS:=+uhttpd
  MAINTAINER:=Sven Schwermer <sven@svenschwermer.de>
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
endef

define Build/Compile/Default

endef
Build/Compile = $(Build/Compile/Default)

define Package/wemo-api/install
	$(INSTALL_DIR) $(1)/www/cgi-bin
	$(INSTALL_BIN) ./files/relay $(1)/www/cgi-bin/
endef

$(eval $(call BuildPackage,wemo-api))
