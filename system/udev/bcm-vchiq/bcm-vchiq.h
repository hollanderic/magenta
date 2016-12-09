
#define VCHIQ_SLOT_SIZE 4096
#define VCHIQ_MAX_SLOTS 128


typedef struct vchiq_slot {
    int magic;
    short version;
    short version_min;
    int slot_zero_size;
    int slot_size;
    int max_slots;
    int max_slots_per_side;
    int platform_data[2];
    VCHIQ_SHARED_STATE_T master;
    VCHIQ_SHARED_STATE_T slave;
    VCHIQ_SLOT_INFO_T slots[VCHIQ_MAX_SLOTS];
} vchiq_slot_zero_t;




typedef struct vchiq_header_struct {
    /* The message identifier - opaque to applications. */
    int32_t msgid;

    /* Size of message data. */
    uint32_t size;

    char data[0];           /* message */
} vchiq_slot_header_t;
