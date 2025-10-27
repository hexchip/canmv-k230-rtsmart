#include "usbh_core.h"
#include "usbh_hid.h"

#include "console.h"

typedef void (*hid_report_handler)(void* context, rt_uint8_t *report, rt_uint16_t nbytes);

struct hid_callback_msg {
    int nbytes;
};

struct hid_callback_context {
    struct usbh_hid *hid_class;
    rt_mq_t mq;
};

struct hid_thread_context {
    struct usbh_hid *hid_class;
};

struct hid_keyboard_handler_context {
    rt_bool_t shift;
    rt_bool_t ctrl;
    rt_bool_t alt;
    rt_bool_t caps_lock;
    rt_bool_t num_lock;
    uint8_t last_keycodes[6];
};

// HID键盘键码到字符串的映射表
static const char* hid_keycode_to_ascii[][2] = {
    [0x04] = {"a", "A"}, [0x05] = {"b", "B"}, [0x06] = {"c", "C"}, [0x07] = {"d", "D"},
    [0x08] = {"e", "E"}, [0x09] = {"f", "F"}, [0x0A] = {"g", "G"}, [0x0B] = {"h", "H"},
    [0x0C] = {"i", "I"}, [0x0D] = {"j", "J"}, [0x0E] = {"k", "K"}, [0x0F] = {"l", "L"},
    [0x10] = {"m", "M"}, [0x11] = {"n", "N"}, [0x12] = {"o", "O"}, [0x13] = {"p", "P"},
    [0x14] = {"q", "Q"}, [0x15] = {"r", "R"}, [0x16] = {"s", "S"}, [0x17] = {"t", "T"},
    [0x18] = {"u", "U"}, [0x19] = {"v", "V"}, [0x1A] = {"w", "W"}, [0x1B] = {"x", "X"},
    [0x1C] = {"y", "Y"}, [0x1D] = {"z", "Z"},
    [0x1E] = {"1", "!"}, [0x1F] = {"2", "@"}, [0x20] = {"3", "#"}, [0x21] = {"4", "$"},
    [0x22] = {"5", "%"}, [0x23] = {"6", "^"}, [0x24] = {"7", "&"}, [0x25] = {"8", "*"},
    [0x26] = {"9", "("}, [0x27] = {"0", ")"},
    [0x2C] = {" ", " "}, [0x2D] = {"-", "_"}, [0x2E] = {"=", "+"}, [0x2F] = {"[", "{"},
    [0x30] = {"]", "}"}, [0x31] = {"\\", "|"}, [0x33] = {";", ":"}, [0x34] = {"'", "\""},
    [0x35] = {"`", "~"}, [0x36] = {",", "<"}, [0x37] = {".", ">"}, [0x38] = {"/", "?"},
    [HID_KBD_USAGE_ENTER] = {"\n", "\n"},
    [HID_KBD_USAGE_DELETE] = {"\b", "\b"},
    [HID_KBD_USAGE_TAB] = {"\t", "\t"},
    [HID_KBD_USAGE_ESCAPE] = {"\x1B", "\x1B"},
};

// 控制字符字符串映射
static const char* hid_keycode_to_ctrl_ascii[] = {
    [0x04] = "\x01", // Ctrl+A → SOH (Start of Heading)
    [0x05] = "\x02", // Ctrl+B → STX (Start of Text)
    [0x06] = "\x03", // Ctrl+C → ETX (End of Text) - 中断信号
    [0x07] = "\x04", // Ctrl+D → EOT (End of Transmission) - 退出
    [0x08] = "\x05", // Ctrl+E → ENQ (Enquiry)
    [0x09] = "\x06", // Ctrl+F → ACK (Acknowledge)
    [0x0A] = "\x07", // Ctrl+G → BEL (Bell) - 响铃
    [0x0B] = "\x08", // Ctrl+H → BS (Backspace)
    [0x0C] = "\x09", // Ctrl+I → HT (Horizontal Tab)
    [0x0D] = "\x0A", // Ctrl+J → LF (Line Feed)
    [0x0E] = "\x0B", // Ctrl+K → VT (Vertical Tab)
    [0x0F] = "\x0C", // Ctrl+L → FF (Form Feed)
    [0x10] = "\x0D", // Ctrl+M → CR (Carriage Return)
    [0x11] = "\x0E", // Ctrl+N → SO (Shift Out)
    [0x12] = "\x0F", // Ctrl+O → SI (Shift In)
    [0x13] = "\x10", // Ctrl+P → DLE (Data Link Escape)
    [0x14] = "\x11", // Ctrl+Q → DC1 (Device Control 1) - XON
    [0x15] = "\x12", // Ctrl+R → DC2 (Device Control 2)
    [0x16] = "\x13", // Ctrl+S → DC3 (Device Control 3) - XOFF
    [0x17] = "\x14", // Ctrl+T → DC4 (Device Control 4)
    [0x18] = "\x15", // Ctrl+U → NAK (Negative Acknowledge)
    [0x19] = "\x16", // Ctrl+V → SYN (Synchronous Idle)
    [0x1A] = "\x17", // Ctrl+W → ETB (End of Transmission Block)
    [0x1B] = "\x18", // Ctrl+X → CAN (Cancel)
    [0x1C] = "\x19", // Ctrl+Y → EM (End of Medium)
    [0x1D] = "\x1A", // Ctrl+Z → SUB (Substitute) - 挂起信号
    [0x2F] = "\x1B", // Ctrl+[ → ESC - 退出键
    [0x31] = "\x1C", // Ctrl+\ → FS (File Separator) - 文件分隔符
    [0x30] = "\x1D", // Ctrl+] → GS (Group Separator) - 组群分隔符
};


static void usbh_hid_callback(void *arg, int nbytes)
{
    struct hid_callback_context *context = (struct hid_callback_context *)arg;

    if (nbytes > 0) {
        struct hid_callback_msg msg = {
            .nbytes = nbytes
        };

        rt_mq_send(context->mq, &msg, sizeof(struct hid_callback_msg));

        usbh_submit_urb(&context->hid_class->intin_urb);
    } else if (nbytes == -USB_ERR_NAK) { /* for dwc2 */
        usbh_submit_urb(&context->hid_class->intin_urb);
    } else {
        struct hid_callback_msg msg = {
            .nbytes = -100
        };
        rt_mq_send(context->mq, &msg, sizeof(struct hid_callback_msg));
    }
}

static void handle_unimplemented_report(void *context, rt_uint8_t *report, rt_uint16_t nbytes) {
    for (size_t i = 0; i < nbytes; i++) {
        USB_LOG_RAW("0x%02x ", report[i]);
    }
    USB_LOG_RAW("nbytes:%d\r\n", nbytes);
}

static const char* keycode_to_ascii(uint8_t keycode, struct hid_keyboard_handler_context *keyboad_context)
{
    // 检查键码是否在我们定义的映射范围内
    if (keycode < 0x04 || keycode > 0x38) {
        return NULL;
    }
    
    // 处理Ctrl组合键
    if (keyboad_context->ctrl) {
        // 检查是否是字母键 (0x04-0x1D)
        if (keycode >= HID_KBD_USAGE_A && keycode < HID_KBD_USAGE_1
            || keycode == HID_KBD_USAGE_LBRACKET
            || keycode == HID_KBD_USAGE_BSLASH
            || keycode == HID_KBD_USAGE_RBRACKET) {
            return hid_keycode_to_ctrl_ascii[keycode];
        }
    }
    
    // 处理Shift状态
    int index = 0;
    if (keyboad_context->shift) {
        index = 1;
    }
    
    // Caps Lock处理 (仅影响字母)
    if (keyboad_context->caps_lock && keycode >= HID_KBD_USAGE_A && keycode < HID_KBD_USAGE_1) {
        index = !index; // 反转大小写
    }
    
    return hid_keycode_to_ascii[keycode][index];
}

static const char* handle_special_keys(uint8_t keycode) {

    switch (keycode) {


        // 方向键
        case HID_KBD_USAGE_LEFT:  return "\x1B[D";
        case HID_KBD_USAGE_RIGHT: return "\x1B[C";
        case HID_KBD_USAGE_DOWN:  return "\x1B[B";
        case HID_KBD_USAGE_UP:    return "\x1B[A";
        
        // 编辑键
        case HID_KBD_USAGE_INSERT: return "\x1B[2~";
        case HID_KBD_USAGE_DELFWD: return "\x1B[3~";
        case HID_KBD_USAGE_HOME: return "\x1B[H";
        case HID_KBD_USAGE_END: return "\x1B[F";
        case HID_KBD_USAGE_PAGEUP: return "\x1B[5~";
        case HID_KBD_USAGE_PAGEDOWN: return "\x1B[6~";
        
        // 功能键 F1-F12
        // case HID_KBD_USAGE_F1: return "\x1B[11~";
        // case HID_KBD_USAGE_F2: return "\x1B[12~";
        // case HID_KBD_USAGE_F3: return "\x1B[13~";
        // case HID_KBD_USAGE_F4: return "\x1B[14~";
        // case HID_KBD_USAGE_F5: return "\x1B[15~";
        // case HID_KBD_USAGE_F6: return "\x1B[17~";
        // case HID_KBD_USAGE_F7: return "\x1B[18~";
        // case HID_KBD_USAGE_F8: return "\x1B[19~";
        // case HID_KBD_USAGE_F9: return "\x1B[20~";
        // case HID_KBD_USAGE_F10: return "\x1B[21~";
        // case HID_KBD_USAGE_F11: return "\x1B[23~";
        // case HID_KBD_USAGE_F12: return "\x1B[24~";
        
        default: return NULL;
    }
}

static const char* keycode_to_output(uint8_t keycode, struct hid_keyboard_handler_context *keyboad_context) {
    switch (keycode) {
        case HID_KBD_USAGE_CAPSLOCK:
            keyboad_context->caps_lock = !keyboad_context->caps_lock;
            return NULL;
        case HID_KBD_USAGE_KPDNUMLOCK:
            keyboad_context->num_lock = !keyboad_context->num_lock;
            return NULL;
    }

    const char* special_sequence = handle_special_keys(keycode);

    if (special_sequence) {
        return special_sequence;
    }

    return keycode_to_ascii(keycode, keyboad_context);
}

static void handle_keyboard_report(void *context, rt_uint8_t *report, rt_uint16_t nbytes) {

    struct hid_keyboard_handler_context *keyboad_context = (struct hid_keyboard_handler_context *)context; 

    uint8_t modifiers = report[0];
    uint8_t *keycodes = &report[2];

    // 更新修饰键状态
    keyboad_context->shift = (modifiers & (HID_MODIFER_LSHIFT | HID_MODIFER_RSHIFT)) != 0;    // 左Shift或右Shift
    keyboad_context->ctrl = (modifiers & (HID_MODIFER_LCTRL | HID_MODIFER_RCTRL)) != 0;     // 左Ctrl或右Ctrl
    keyboad_context->alt = (modifiers & (HID_MODIFER_LALT | HID_MODIFER_RALT)) != 0;      // 左Alt或右Alt
    
    // 处理每个按键
    for (int i = 0; i < 6; i++) {
        uint8_t keycode = keycodes[i];
        if (keycode == HID_KBD_USAGE_NONE) continue; // 空键码
        
        // 检查这个键是否在上一次报告中（即是否已经处理过）
        // TODO 期望HID报告的顺序不变，待观察
        if (keyboad_context->last_keycodes[i] == keycode) {
            continue;
        }

        const char *str = keycode_to_output(keycode, keyboad_context);

        // rt_kprintf("\nkeycode_to_output=%s\n", str);

        if (str) {
            struct tty_struct *console = console_tty_get();
            size_t len = rt_strlen(str);
            char buffer[len + 1];
            rt_strcpy(buffer, str);
            console->ldisc->ops->receive_buf((struct tty_struct *)console, buffer, rt_strlen(str));
        }
    }

    rt_memcpy(keyboad_context->last_keycodes, keycodes, 6);
}

static hid_report_handler get_hid_report_handler(rt_uint8_t interface_protocol) {
    if (interface_protocol == HID_PROTOCOL_KEYBOARD) {
        return handle_keyboard_report;
    }
    else {
        return handle_unimplemented_report;
    }
}

static void* create_hid_handler_context(rt_uint8_t interface_protocol) {
    if (interface_protocol == HID_PROTOCOL_KEYBOARD) {
        struct hid_keyboard_handler_context *context = rt_malloc(sizeof(struct hid_keyboard_handler_context));
        memset(context, 0, sizeof(struct hid_keyboard_handler_context));
        return context;
    }
    else {
        return RT_NULL;
    }
}

static void destroy_hid_handler_context(void *context, rt_uint8_t interface_protocol) {
    if (interface_protocol == HID_PROTOCOL_KEYBOARD) {
        struct hid_keyboard_handler_context *hid_keyboard_handler_context = (struct hid_keyboard_handler_context *)context;
        rt_free(hid_keyboard_handler_context);
    }
}

static void usbh_hid_thread(void *argument) {
    struct usbh_hid *hid_class = (struct usbh_hid *)argument;

    uint8_t *transfer_buffer = rt_malloc(128);

    rt_mq_t mq = rt_mq_create(rt_thread_self()->name, sizeof(struct hid_callback_msg), 64, RT_IPC_FLAG_PRIO);

    struct hid_callback_context *hid_callback_context = rt_malloc(sizeof(struct hid_callback_context));
    hid_callback_context->hid_class = hid_class;
    hid_callback_context->mq = mq;

    usbh_int_urb_fill(&hid_class->intin_urb, hid_class->hport, hid_class->intin, transfer_buffer, hid_class->intin->wMaxPacketSize, 0, usbh_hid_callback, hid_callback_context);
    int ret = usbh_submit_urb(&hid_class->intin_urb);
    if (ret < 0) {
        rt_kprintf("urb ret = %d\n", ret);
        goto delete;
    }

    struct hid_callback_msg msg;
    rt_uint8_t interface_protocol = hid_class->hport->config.intf[hid_class->intf].altsetting->intf_desc.bInterfaceProtocol;
    hid_report_handler hid_report_handler = get_hid_report_handler(interface_protocol);
    void *hid_handler_context = create_hid_handler_context(interface_protocol);
    while (hid_class->hport->connected) {
        if (rt_mq_recv(mq, &msg, sizeof(struct hid_callback_msg), RT_WAITING_FOREVER) == RT_EOK) {
            if (msg.nbytes == -100) {
                break;
            }
            hid_report_handler(hid_handler_context, transfer_buffer, msg.nbytes);
        }
    }
    
delete:
    destroy_hid_handler_context(hid_handler_context, interface_protocol);
    rt_free(hid_callback_context);
    rt_mq_delete(mq);
    rt_free(transfer_buffer);
    usb_osal_thread_delete(NULL);
}

void usbh_hid_run(struct usbh_hid *hid_class) {
    char thread_name[CONFIG_USBHOST_DEV_NAMELEN];

    char *devname = hid_class->hport->config.intf[hid_class->intf].devname;
    const char *prefix = "/dev/";
    size_t prefix_len = strlen(prefix);
    if (strncmp(devname, prefix, prefix_len) == 0) {
        devname = devname + prefix_len;
    }

    rt_snprintf(thread_name, CONFIG_USBHOST_DEV_NAMELEN, "%s", devname);
    usb_osal_thread_create(thread_name, 2048, CONFIG_USBHOST_PSC_PRIO + 1, usbh_hid_thread, hid_class);
}