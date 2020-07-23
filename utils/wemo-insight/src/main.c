#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libubox/ulog.h>
#include <libubox/uloop.h>
#include <libubus.h>

#include "insight.h"

#define xstr(a) str(a)
#define str(a) #a

#define add_field(b, d, field) \
    blobmsg_add_double(&b, xstr(field), d->field)

struct priv
{
    struct insight_state *insight;
    struct ubus_auto_conn ubus_conn;
};

static struct priv *get_priv(struct ubus_context *ctx)
{
    struct ubus_auto_conn *ubus_conn = container_of(ctx, struct ubus_auto_conn, ctx);
    return container_of(ubus_conn, struct priv, ubus_conn);
}

static int insight_info(struct ubus_context *ctx, struct ubus_object *obj,
                        struct ubus_request_data *req, const char *method,
                        struct blob_attr *msg)
{
    struct blob_buf b = {0};
    blob_buf_init(&b, 0);

    struct priv *priv = get_priv(ctx);
    const struct insight_data *d = insight_borrow_data(priv->insight);
    
    add_field(b, d, int_temperature);
    add_field(b, d, ext_temperature);
    add_field(b, d, rms_voltage);
    add_field(b, d, rms_current);
    add_field(b, d, active_power);
    add_field(b, d, average_power);
    add_field(b, d, power_factor);
    add_field(b, d, line_frequency);
    add_field(b, d, active_energy);
    
    insight_return_data(priv->insight);
    
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

    struct priv priv = {
        .insight = insight_open(device),
        .ubus_conn = {
            .path = ubus_socket,
            .cb = ubus_connect_handler,
        },
    };
    if (priv.insight == NULL)
        return -1;

    ubus_auto_connect(&priv.ubus_conn);
    uloop_run();
    uloop_done();

    insight_free(priv.insight);

    ULOG_INFO("Shutting down\n");
    return 0;
}
