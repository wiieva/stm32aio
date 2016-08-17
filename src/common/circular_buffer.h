#pragma once

typedef struct circular_buffer 
{
    uint32_t buf_size;
    uint32_t head;
    uint32_t tail;
    uint32_t full;
    uint8_t *buf;
    uint16_t write_cnt;
    uint16_t read_cnt;
} circular_buffer;


int cbuf_init (circular_buffer *_cb,uint32_t _buf_size);
void cbuf_destroy (circular_buffer *_cb);

uint32_t cbuf_write (circular_buffer *_cb, uint8_t *_p_ins, uint32_t _s_ins);
uint32_t cbuf_read (circular_buffer *_cb, uint8_t *_p_ins,uint32_t _s_ins);
uint32_t cbuf_free (circular_buffer *_cb,uint32_t _s_free,int from_back);
void cbuf_clear (circular_buffer *_cb);

inline static uint32_t cbuf_count (circular_buffer *_cb) {
    int D = _cb->head - _cb->tail;
    if (D < 0 || (D==0 && _cb->full))
         D += _cb->buf_size;
    return D;
}

inline static uint32_t cbuf_free_count (circular_buffer *_cb) {
    return (_cb->buf_size - cbuf_count (_cb));
}

inline static uint32_t cbuf_write_ptr (circular_buffer *_cb, uint8_t **_p_ins, uint32_t _s_ins) {
    uint16_t cnt = ((_cb->head >= _cb->tail&&!_cb->full)?_cb->buf_size:_cb->tail) - _cb->head;

    if (cnt > _s_ins)
        cnt = _s_ins;

    *_p_ins = _cb->buf + _cb->head;
    _cb->write_cnt = cnt;
    return cnt;
}

inline static uint32_t cbuf_write_commit (circular_buffer *_cb) {
    uint16_t cnt = _cb->write_cnt;
    _cb->write_cnt = 0;
    if (cnt) {
        _cb->head = (_cb->head + cnt) % _cb->buf_size;
        _cb->full = (_cb->head == _cb->tail);
    }
    return cnt;
}

inline static uint32_t cbuf_read_ptr (circular_buffer *_cb, uint8_t **_p_ins, uint32_t _s_ins) {
    uint16_t cnt = ((_cb->tail > _cb->head||_cb->full)?_cb->buf_size:_cb->head) - _cb->tail;

    if (cnt > _s_ins)
        cnt = _s_ins;

    *_p_ins = _cb->buf + _cb->tail;
    _cb->read_cnt = cnt;
    return cnt;
}

inline static uint32_t cbuf_read_commit (circular_buffer *_cb)
{
    uint16_t cnt = _cb->read_cnt;
    _cb->read_cnt = 0;
    if (cnt) {
        _cb->tail = (_cb->tail + cnt) % _cb->buf_size;
        _cb->full = 0;
    }
    return cnt;
}

inline static uint32_t cbuf_put_char (circular_buffer *_cb, uint8_t data) {
    if (!cbuf_free_count(_cb))
        return 0;

    _cb->buf[_cb->head] = data;

    _cb->head = (_cb->head + 1) % _cb->buf_size;
    _cb->full = (_cb->head==_cb->tail);
    return 1;
}
