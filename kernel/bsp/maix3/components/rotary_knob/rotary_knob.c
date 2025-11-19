#include "rotary_knob.h"

#include "console.h"

#define DBG_TAG    "ROTARY_KNOB"
#define DBG_LVL    DBG_WARNING
#include <rtdbg.h>

struct rotary_knob {
    int clk_pin;
    int dt_pin;
    int sw_pin;
    rt_device_t dev;
    uint32_t index;
    rt_list_t list;
};

static rt_list_t s_rotary_knob_list = RT_LIST_OBJECT_INIT(s_rotary_knob_list);

static rotary_knob_t * find_rotary_knob(rotary_knob_config_t *cfg) {
    rotary_knob_t *entry = RT_NULL;

    rt_list_for_each_entry(entry, &s_rotary_knob_list, list) {
        if (entry->clk_pin == cfg->clk_pin
            && entry->dt_pin == cfg->dt_pin
            && entry->sw_pin == cfg->sw_pin) {
            return entry;
        }
    }

    return RT_NULL;
}

static uint64_t s_index_in_use_flag = 0;

static int allocate_rotary_knob_index() {
    for (uint8_t index = 0; index < (sizeof(s_index_in_use_flag) * 8); index++) {
        if ((s_index_in_use_flag & (1ULL << index)) == 0) {
            s_index_in_use_flag |= (1ULL << index);
            return index;
        }
    }

    return -1;
}

static void free_rotary_knob_index(int index) {
    if (index < 0) {
        return;
    }

    if (index < (sizeof(s_index_in_use_flag) * 8)) {
        s_index_in_use_flag &= ~(1ULL << index);
    }
}

rotary_knob_t * rotary_knob_create(rotary_knob_config_t *cfg) {

    rotary_knob_t *rotary_knob = find_rotary_knob(cfg);

    if (rotary_knob != RT_NULL) {
        LOG_W("rotary_knob already existed! index = %d clk = %d, dt = %d, sw = %s",
             rotary_knob->index, 
             rotary_knob->clk_pin, 
             rotary_knob->dt_pin, 
             rotary_knob->sw_pin);

        return rotary_knob;
    }

    int rotary_knob_index = allocate_rotary_knob_index();
    if (rotary_knob_index < 0) {
        LOG_E("allocate_rotary_knob_index failed!");
        rt_set_errno(RT_EBUSY);
        return RT_NULL;
    }

    encoder_dev_cfg_t encoder_dev_cfg = {
        .index = rotary_knob_index,
        .cfg.clk_pin = cfg->clk_pin,
        .cfg.dt_pin = cfg->dt_pin,
        .cfg.sw_pin = cfg->sw_pin,
    };

    int err = encoder_dev_create(&encoder_dev_cfg);

    if (err) {
        LOG_E("encoder_dev_create failed! err = %d", err);
        rt_set_errno(err);
        return RT_NULL;
    }

    rotary_knob = rt_malloc(sizeof(rotary_knob_t));
    if (rotary_knob == RT_NULL) {
        LOG_E("malloc rotary_knob_t failed!");
        rt_set_errno(RT_ENOMEM);
        encoder_dev_delete(rotary_knob_index);
        return RT_NULL;
    }

    rotary_knob->index = rotary_knob_index;
    rotary_knob->clk_pin = cfg->clk_pin;
    rotary_knob->dt_pin = cfg->dt_pin;
    rotary_knob->sw_pin = cfg->sw_pin;

    rt_list_init(&rotary_knob->list);

    char dev_name[RT_NAME_MAX];
    rt_snprintf(dev_name, sizeof(dev_name), "encoder%d", rotary_knob_index);
    rt_device_t dev = rt_device_find(dev_name);
    if (dev == RT_NULL) {
        LOG_E("rt_device_find %s failed!", dev_name);
        rt_set_errno(RT_ERROR);
        rotary_knob_destroy(rotary_knob);
    }
    rotary_knob->dev = dev;

    rt_list_insert_after(&s_rotary_knob_list, &rotary_knob->list);

    return rotary_knob;
}

void rotary_knob_destroy(rotary_knob_t *rotary_knob) {
    if (rotary_knob == RT_NULL) {
        return;
    }

    encoder_dev_delete(rotary_knob->index);

    rt_list_remove(&rotary_knob->list);

    rt_free(rotary_knob);
}


static rt_thread_t s_thread;

static void thread_entry(void *context) {
    rotary_knob_t *rotary_knob = context;
    struct tty_struct *console = console_tty_get();
    while (1) {
        int32_t timeout = RT_WAITING_FOREVER;
        rt_err_t err = rt_device_control(rotary_knob->dev, ENCODER_CMD_WAIT_DATA, &timeout);

        rt_device_t uart2 = rt_device_find("UART2");

        if (err == RT_EOK) {
            struct encoder_data data;
            err = rt_device_control(rotary_knob->dev, ENCODER_CMD_GET_DATA, &data);
            if (err) {
                LOG_E("rotary_knob get data failed! err = %d", err);
                continue;
            }

            if (data.direction == 0) {
                if (data.button_state == 1) {
                    console->ldisc->ops->receive_buf((struct tty_struct *)console, "\x1B\x5b\x3f\x33", 6);
                }
            }
            else if (data.direction == 1) {
                console->ldisc->ops->receive_buf((struct tty_struct *)console, "\x1B\x5b\x3f\x31", 6);
            }
            else if (data.direction == 2) {
                console->ldisc->ops->receive_buf((struct tty_struct *)console, "\x1B\x5b\x3f\x32", 6);
            }

            rt_device_write(uart2, 0, "rotary_knob state change!", 26 * sizeof(char));
            LOG_D("rotary_knob button_state = %d ", data.button_state);
            LOG_D("rotary_knob delta = %d ", data.delta);
            LOG_D("rotary_knob direction = %d ", data.direction);
            LOG_D("rotary_knob timestamp = %d ", data.timestamp);
            LOG_D("rotary_knob total_count = %d ", data.total_count);
        }
        else {
            LOG_E("rotary_knob wait data failed! err = %d", err);
            break;
        }
    }
    
}

static int rotary_knob_thread_init() {
    rotary_knob_config_t cfg = {
        .clk_pin = 3,
        .dt_pin = 5,
        .sw_pin = 6
    };

    rotary_knob_t *rotary_knob = rotary_knob_create(&cfg);

    if (rotary_knob == RT_NULL) {
        return errno;
    }

    s_thread = rt_thread_create("sys_rotary_knob", thread_entry, rotary_knob, 1024 * 4, 9, 10);
    if (s_thread == RT_NULL) {
        return -1;
    }

    rt_err_t err = rt_thread_startup(s_thread);

    return err;
}
INIT_COMPONENT_EXPORT(rotary_knob_thread_init);