#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <libubox/ulog.h>
#include <libubox/uloop.h>
#include <libubus.h>

#include "insight.h"

static int insight_info(struct ubus_context *ctx, struct ubus_object *obj,
                        struct ubus_request_data *req, const char *method,
                        struct blob_attr *msg)
{
    struct blob_buf b = {0};
    blob_buf_init(&b, 0);
    blobmsg_add_u32(&b, "test", 0xdeadbeef);
    ubus_send_reply(ctx, req, b.head);
    return UBUS_STATUS_OK;
}

static const struct ubus_method insight_methods[] = {
    UBUS_METHOD_NOARG("info", insight_info),
};

static struct ubus_object_type insight_object_type =
    UBUS_OBJECT_TYPE("wemo-insight", insight_methods);

static struct ubus_object insight_object = {
    .name = "wemo-insight",
    .type = &insight_object_type,
    .methods = insight_methods,
    .n_methods = ARRAY_SIZE(insight_methods),
};

static void ubus_connect_handler(struct ubus_context *ctx)
{
    int ret = ubus_add_object(ctx, &insight_object);
    if (ret)
        ULOG_ERR("Failed to add bus object: %s\n", ubus_strerror(ret));
    else
        ULOG_INFO("Bus object added\n");
}

static int usage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s [options] <device>\n"
            "Options:\n"
            "  -s <path>  Path to ubus socket\n"
            "  -S         Print messages to stdout\n"
            "\n",
            prog);
    return -1;
}

int main(int argc, char **argv)
{
    int ch;
    char *ubus_socket = NULL;
    int ulog_channels = ULOG_KMSG;

    signal(SIGPIPE, SIG_IGN);

    while ((ch = getopt(argc, argv, "s:S")) != -1)
    {
        switch (ch)
        {
        case 's':
            ubus_socket = optarg;
            break;
        case 'S':
            ulog_channels = ULOG_STDIO;
            break;
        default:
            return usage(argv[0]);
        }
    }

    if (argc - optind < 1)
    {
        fprintf(stderr, "ERROR: missing device parameter\n");
        return usage(argv[0]);
    }
    const char *device = argv[optind];

    ulog_open(ulog_channels, LOG_DAEMON, "wemo-insight");

    uloop_init();
    struct ubus_auto_conn conn = {
        .path = ubus_socket,
        .cb = ubus_connect_handler,
    };
    ubus_auto_connect(&conn);

    struct insight_data *h = insight_open(device);
    if (h == NULL)
        return -1;

    uloop_run();
    uloop_done();

    insight_free(h);

    ULOG_INFO("Shutting down\n");
    return 0;
}
