#include "circular_buf.h"

#include <stdint.h>

#include "stm32g4xx_hal.h"

#include "circular_buf_definition.h"

static void __circular_buf_advance_pointer(circular_buf_handle cbuf);
static void __circular_buf_retreat_pointer(circular_buf_handle cbuf);
static uint8_t* __circular_buf_get_elem(circular_buf_handle cbuf);

static struct circular_buf cb;

circular_buf_handle circular_buffer = &cb;

void circular_buf_init(circular_buf_handle cbuf, uint8_t* buf, uint32_t size) {
  assert_param(cbuf && buf && size);

  cbuf->buf = buf;
  cbuf->capacity = size;
  cbuf->element_size = 1;

  circular_buf_reset(cbuf);
}

void circular_buf_reset(circular_buf_handle cbuf) {
  assert_param(cbuf);
  cbuf->head = 0;
  cbuf->tail = 0;
  cbuf->is_full = 0;
  cbuf->is_elem_ready_counter = 0;
  cbuf->element_size = 1;
}

void circular_buf_put(circular_buf_handle cbuf, uint8_t byte) {
  assert_param(cbuf && cbuf->buf);
  cbuf->buf[cbuf->head] = byte;

  __circular_buf_advance_pointer(cbuf);

  if (cbuf->is_elem_ready_counter == cbuf->element_size) {
    cbuf->is_elem_ready_counter = 0;
    circular_buf_element_ready_callback(__circular_buf_get_elem(cbuf));
  }
}

int8_t circular_buf_get(circular_buf_handle cbuf, uint8_t* data) {
  assert_param(cbuf && data && cbuf->buf);

  int8_t status = -1;

  if (!circular_buf_is_empty(cbuf)) {
    *data = cbuf->buf[cbuf->tail];
    __circular_buf_retreat_pointer(cbuf);

    status = 0;
  }
  return status;
}

void circular_buf_write(circular_buf_handle cbuf, uint8_t* source,
                        uint32_t count) {
  assert_param(cbuf && source);
  assert_param(count <= cbuf->capacity);

  for (uint32_t i = 0; i < count; ++i) {
    circular_buf_put(cbuf, source[i]);
  }
}

void circular_buf_read(circular_buf_handle cbuf, uint8_t* target,
                       uint32_t count) {
  assert_param(cbuf && target);
  assert_param(count <= cbuf->capacity);

  for (uint32_t i = 0; i < count; ++i) {
    circular_buf_get(cbuf, &target[i]);
  }
}

void circular_buf_set_element_size(circular_buf_handle cbuf, uint32_t size) {
  assert_param(cbuf && size);
  cbuf->element_size = size;
}

uint32_t circular_buf_get_element_size(circular_buf_handle cbuf) {
  assert_param(cbuf);
  return cbuf->element_size;
}

uint8_t circular_buf_is_empty(circular_buf_handle cbuf) {
  return (!cbuf->is_full && (cbuf->head == cbuf->tail));
}

uint8_t circular_buf_is_full(circular_buf_handle cbuf) {
  assert_param(cbuf);
  return cbuf->is_full;
}

uint32_t circular_buf_capacity(circular_buf_handle cbuf) {
  assert_param(cbuf);
  return cbuf->capacity;
}

uint32_t circular_buf_size(circular_buf_handle cbuf) {
  assert_param(cbuf);
  uint32_t size = cbuf->capacity;
  if (!cbuf->is_full) {
    if (cbuf->head >= cbuf->tail) {
      size = cbuf->head - cbuf->tail;
    } else {
      size = cbuf->capacity + cbuf->head - cbuf->tail;
    }
  }
  return size;
}

static void __circular_buf_advance_pointer(circular_buf_handle cbuf) {
  assert_param(cbuf);

  if (cbuf->is_full) cbuf->tail = (++cbuf->tail) % cbuf->capacity;

  cbuf->head = (++cbuf->head) % cbuf->capacity;
  cbuf->is_full = (cbuf->head == cbuf->tail);
  ++cbuf->is_elem_ready_counter;
}

static void __circular_buf_retreat_pointer(circular_buf_handle cbuf) {
  assert_param(cbuf);
  cbuf->is_full = 0;
  cbuf->tail = (++cbuf->tail) % cbuf->capacity;
}

uint8_t* __circular_buf_get_elem(circular_buf_handle cbuf) {
  assert_param(cbuf);
  uint32_t index = cbuf->head - cbuf->element_size;

  if (cbuf->head < cbuf->element_size) {
    index = cbuf->capacity - cbuf->element_size;
  }
  return &cbuf->buf[index];
}
