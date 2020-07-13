#ifndef CIRCULAR_BUF_GUARD
#define CIRCULAR_BUF_GUARD

#include <stdint.h>

struct circular_buf;

typedef struct circular_buf* circular_buf_handle;

void circular_buf_init(circular_buf_handle cbuf, uint8_t* buf, uint32_t size);

void circular_buf_reset(circular_buf_handle cbuf);

void circular_buf_put(circular_buf_handle cbuf, uint8_t byte);

int8_t circular_buf_get(circular_buf_handle cbuf, uint8_t* data);

void circular_buf_write(circular_buf_handle cbuf, uint8_t* source,
                        uint32_t count);

void circular_buf_read(circular_buf_handle cbuf, uint8_t* target,
                       uint32_t count);

void circular_buf_set_element_size(circular_buf_handle cbuf, uint32_t size);

uint32_t circular_buf_get_element_size(circular_buf_handle cbuf);

uint8_t circular_buf_is_empty(circular_buf_handle cbuf);

uint8_t circular_buf_is_full(circular_buf_handle cbuf);

uint32_t circular_buf_capacity(circular_buf_handle cbuf);

uint32_t circular_buf_size(circular_buf_handle cbuf);
/* To be implemented by user*/
void circular_buf_element_ready_callback(uint8_t* pElem);

#endif  // !CIRCULAR_BUF_GUARD
