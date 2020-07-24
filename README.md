# Wemo Insight OpenWrt package feed

Use with my [OpenWrt fork][1].

## wemo-insight
Allows to query the "Insight" data, i.e. power consumption data, either via ubus
(`ubus call wemo-insight info`) on the target or via HTTP
(`curl HOSTNAME/cgi-bin/insight`).

## wemo-button
Allows to switch the relay via the touch field on the device.

## wemo-api
Allows to switch the relay and query the relay state via HTTP:

```sh
# query state:
curl HOSTNAME/cgi-bin/relay
# switch state:
curl -d on HOSTNAME/cgi-bin/relay
curl -d off HOSTNAME/cgi-bin/relay
```

[1]:https://github.com/svenschwermer/openwrt/tree/f7c029
