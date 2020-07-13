#ifndef CIRCULAR_BUF_DEFINITIONS_GUARD
#define CIRCULAR_BUF_DEFINITIONS_GUARD

struct circular_buf {
  uint8_t* buf;
  uint32_t head;
  uint32_t tail;
  uint32_t capacity;
  uint32_t element_size;
  uint32_t is_elem_ready_counter;
  uint8_t is_full;
};

#endif  // !CIRCULAR_BUF_DEFINITIONS_GUARD
