#include <string.h>
#include <malloc.h>
#include <stdint.h>

#include "circular_buffer.h"

#define min(a,b) ((a)<(b)?(a):(b))

int cbuf_init (circular_buffer *_cb,uint32_t _buf_size) {
    _cb->head = 0;
    _cb->tail = 0;
    _cb->full = 0;
    _cb->buf_size = _buf_size;
    _cb->write_cnt = _cb->read_cnt = 0;
    _cb->buf = (uint8_t*)malloc (_buf_size);
    return (_cb->buf == 0);
}

void cbuf_destroy (circular_buffer *_cb) {
    if (_cb->buf)
        free (_cb->buf);
    _cb->buf = 0;
    _cb->head = 0;
    _cb->tail = 0;
    _cb->full = 0;
    _cb->buf_size = 0;
}

uint32_t cbuf_write (circular_buffer *_cb, uint8_t *_p_ins, uint32_t _s_ins) {
    if (_s_ins > cbuf_free_count(_cb))
        _s_ins = cbuf_free_count (_cb);

    if (!_s_ins)
        return 0;

    uint32_t lSize = _cb->buf_size - _cb->head;

    memcpy (_cb->buf+_cb->head,_p_ins, min (_s_ins, lSize));

    if (_s_ins > lSize)
        memcpy (_cb->buf,_p_ins + lSize, (_s_ins - lSize));

    _cb->head = (_cb->head + _s_ins) % _cb->buf_size;
    _cb->full = (_cb->head==_cb->tail);
    return _s_ins;
}

uint32_t cbuf_read (circular_buffer *_cb, uint8_t *_p_ins,uint32_t _s_ins) {
    if (_s_ins > cbuf_count(_cb))
        _s_ins = cbuf_count (_cb);

    if (!_s_ins)
        return 0;

    uint32_t lSize = _cb->buf_size - _cb->tail;

    memcpy (_p_ins, _cb->buf+_cb->tail, min (_s_ins, lSize));

    if (_s_ins > lSize)
        memcpy (_p_ins + lSize, _cb->buf,(_s_ins - lSize));

    _cb->tail = (_cb->tail + _s_ins) % _cb->buf_size;
    _cb->full = 0;
    return _s_ins;
}

uint32_t cbuf_free (circular_buffer *_cb,uint32_t _s_free,int from_back) {
    if (_s_free > cbuf_count(_cb))
        _s_free = cbuf_count (_cb);

    if (from_back)
        _cb->head = (_cb->head + _cb->buf_size - _s_free) % _cb->buf_size;
    else
        _cb->tail = (_cb->tail + _s_free) % _cb->buf_size;
    _cb->full = 0;
    return _s_free;
}

void cbuf_clear (circular_buffer *_cb) {
    _cb->head = 0;
    _cb->tail = 0;
    _cb->full = 0;
}
