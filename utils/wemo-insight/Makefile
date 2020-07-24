#
# Copyright (C) 2020 Sven Schwermer
#
# This is free software, licensed under the GNU General Public License v2.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=wemo-insight
PKG_VERSION:=0.1
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0

include $(INCLUDE_DIR)/package.mk

define Package/wemo-insight
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Get insight into home energy usage
  DEPENDS:=+libubox +libubus +uhttpd
  MAINTAINER:=Sven Schwermer <sven@svenschwermer.de>
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Package/wemo-insight/install
	$(INSTALL_DIR) $(1)/usr/sbin $(1)/etc/init.d $(1)/www/cgi-bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/wemo-insight $(1)/usr/sbin/
	$(INSTALL_BIN) ./files/wemo-insight.init $(1)/etc/init.d/wemo-insight
	$(INSTALL_BIN) ./files/insight $(1)/www/cgi-bin/
endef

$(eval $(call BuildPackage,wemo-insight))
