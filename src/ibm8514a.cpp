// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
// Adapted for ES40 emulator 

#include "StdAfx.h"
#include "ibm8514a.h"
#include "VGA.h"        

#ifndef IBM8514A_LOG
#define IBM8514A_LOG 1
#endif
#define LOG(...) do { if (IBM8514A_LOG) printf(__VA_ARGS__); } while(0)

#define IBM8514_LINE_LENGTH (m_vga->offset())


ibm8514a_device::ibm8514a_device()
    : m_vga(nullptr)
{
    memset(&ibm8514, 0, sizeof(ibm8514));
}

void ibm8514a_device::device_reset()
{
    memset(&ibm8514, 0, sizeof(ibm8514));
    ibm8514.read_mask = 0x00000000;
    ibm8514.write_mask = 0xffffffff;
}

void ibm8514a_device::ibm8514_write_fg(uint32_t offset)
{
    offset %= m_vga->vga.svga_intf.vram_size;
    uint8_t dst = m_vga->mem_linear_r(offset);
    uint8_t src = 0;

    // check clipping rectangle
    if ((ibm8514.current_cmd & 0xe000) == 0xc000)  
    {
        if (ibm8514.dest_x < ibm8514.scissors_left || ibm8514.dest_x > ibm8514.scissors_right || ibm8514.dest_y < ibm8514.scissors_top || ibm8514.dest_y > ibm8514.scissors_bottom)
            return;  // do nothing
    }
    else
    {
        if (ibm8514.curr_x < ibm8514.scissors_left || ibm8514.curr_x > ibm8514.scissors_right || ibm8514.curr_y < ibm8514.scissors_top || ibm8514.curr_y > ibm8514.scissors_bottom)
            return;  // do nothing
    }

    // determine source
    switch (ibm8514.fgmix & 0x0060)
    {
    case 0x0000:
        src = ibm8514.bgcolour;
        break;
    case 0x0020:
        src = ibm8514.fgcolour;
        break;
    case 0x0040:
    {
        u32 shift_values[4] = { 0, 8, 16, 24 };
        src = (ibm8514.pixel_xfer >> shift_values[(ibm8514.curr_x - ibm8514.prev_x) & 3]) & 0xff;
        break;
    }
    case 0x0060:
        src = m_vga->mem_linear_r(((ibm8514.curr_y * IBM8514_LINE_LENGTH) + ibm8514.curr_x));
        break;
    }

    // write the data
    switch (ibm8514.fgmix & 0x000f)
    {
    case 0x0000: m_vga->mem_linear_w(offset, ~dst);           break;
    case 0x0001: m_vga->mem_linear_w(offset, 0x00);           break;
    case 0x0002: m_vga->mem_linear_w(offset, 0xff);           break;
    case 0x0003: m_vga->mem_linear_w(offset, dst);            break;
    case 0x0004: m_vga->mem_linear_w(offset, ~src);           break;
    case 0x0005: m_vga->mem_linear_w(offset, src ^ dst);      break;
    case 0x0006: m_vga->mem_linear_w(offset, ~(src ^ dst));   break;
    case 0x0007: m_vga->mem_linear_w(offset, src);            break;
    case 0x0008: m_vga->mem_linear_w(offset, ~(src & dst));   break;
    case 0x0009: m_vga->mem_linear_w(offset, (~src) | dst);   break;
    case 0x000a: m_vga->mem_linear_w(offset, src | (~dst));   break;
    case 0x000b: m_vga->mem_linear_w(offset, src | dst);      break;
    case 0x000c: m_vga->mem_linear_w(offset, src & dst);      break;
    case 0x000d: m_vga->mem_linear_w(offset, src & (~dst));   break;
    case 0x000e: m_vga->mem_linear_w(offset, (~src) & dst);   break;
    case 0x000f: m_vga->mem_linear_w(offset, ~(src | dst));   break;
    }
}

void ibm8514a_device::ibm8514_write_bg(uint32_t offset)
{
    offset %= m_vga->vga.svga_intf.vram_size;
    uint8_t dst = m_vga->mem_linear_r(offset);
    uint8_t src = 0;

    // check clipping rectangle
    if ((ibm8514.current_cmd & 0xe000) == 0xc000) 
    {
        if (ibm8514.dest_x < ibm8514.scissors_left || ibm8514.dest_x > ibm8514.scissors_right || ibm8514.dest_y < ibm8514.scissors_top || ibm8514.dest_y > ibm8514.scissors_bottom)
            return;  // do nothing
    }
    else
        if (ibm8514.curr_x < ibm8514.scissors_left || ibm8514.curr_x > ibm8514.scissors_right || ibm8514.curr_y < ibm8514.scissors_top || ibm8514.curr_y > ibm8514.scissors_bottom)
            return;  // do nothing

    // determine source
    switch (ibm8514.bgmix & 0x0060)
    {
    case 0x0000:
        src = ibm8514.bgcolour;
        break;
    case 0x0020:
        src = ibm8514.fgcolour;
        break;
    case 0x0040:
        src = ibm8514.pixel_xfer;
        break;
    case 0x0060:
        src = m_vga->mem_linear_r(((ibm8514.curr_y * IBM8514_LINE_LENGTH) + ibm8514.curr_x));
        break;
    }

    // write the data
    switch (ibm8514.bgmix & 0x000f)
    {
    case 0x0000: m_vga->mem_linear_w(offset, ~dst);           break;
    case 0x0001: m_vga->mem_linear_w(offset, 0x00);           break;
    case 0x0002: m_vga->mem_linear_w(offset, 0xff);           break;
    case 0x0003: m_vga->mem_linear_w(offset, dst);            break;
    case 0x0004: m_vga->mem_linear_w(offset, ~src);           break;
    case 0x0005: m_vga->mem_linear_w(offset, src ^ dst);      break;
    case 0x0006: m_vga->mem_linear_w(offset, ~(src ^ dst));   break;
    case 0x0007: m_vga->mem_linear_w(offset, src);            break;
    case 0x0008: m_vga->mem_linear_w(offset, ~(src & dst));   break;
    case 0x0009: m_vga->mem_linear_w(offset, (~src) | dst);   break;
    case 0x000a: m_vga->mem_linear_w(offset, src | (~dst));   break;
    case 0x000b: m_vga->mem_linear_w(offset, src | dst);      break;
    case 0x000c: m_vga->mem_linear_w(offset, src & dst);      break;
    case 0x000d: m_vga->mem_linear_w(offset, src & (~dst));   break;
    case 0x000e: m_vga->mem_linear_w(offset, (~src) & dst);   break;
    case 0x000f: m_vga->mem_linear_w(offset, ~(src | dst));   break;
    }
}

void ibm8514a_device::ibm8514_write(uint32_t offset, uint32_t src)
{
    int data_size = 8;
    uint32_t xfer;

    switch (ibm8514.pixel_control & 0x00c0)
    {
    case 0x0000:  
        ibm8514_write_fg(offset);
        break;
    case 0x0040:  // fixed pattern (?)
        // TODO
        break;
    case 0x0080:  // use pixel transfer register
        if (ibm8514.bus_size == 0)  // 8-bit
            data_size = 8;
        if (ibm8514.bus_size == 1)  // 16-bit
            data_size = 16;
        if (ibm8514.bus_size == 2)  // 32-bit
            data_size = 32;
        if ((ibm8514.current_cmd & 0x1000) && (data_size != 8))
        {
            xfer = ((ibm8514.pixel_xfer & 0x000000ff) << 8) | ((ibm8514.pixel_xfer & 0x0000ff00) >> 8)
                | ((ibm8514.pixel_xfer & 0x00ff0000) << 8) | ((ibm8514.pixel_xfer & 0xff000000) >> 8);
        }
        else
            xfer = ibm8514.pixel_xfer;
        if (ibm8514.current_cmd & 0x0002)
        {
            if ((xfer & ((1 << (data_size - 1)) >> ibm8514.src_x)) != 0)
                ibm8514_write_fg(offset);
            else
                ibm8514_write_bg(offset);
        }
        else
        {
            ibm8514_write_fg(offset);
        }
        ibm8514.src_x++;
        if (ibm8514.src_x >= data_size)
            ibm8514.src_x = 0;
        break;
    case 0x00c0:  // use source plane
        if (m_vga->mem_linear_r(src) != 0x00)
            ibm8514_write_fg(offset);
        else
            ibm8514_write_bg(offset);
        break;
    }
}

uint16_t ibm8514a_device::ibm8514_line_error_r()
{
    return ibm8514.line_errorterm;
}

void ibm8514a_device::ibm8514_line_error_w(uint16_t data)
{
    ibm8514.line_errorterm = data;
    LOG("8514/A: Line Parameter/Error Term write %04x\n", data);
}

uint16_t ibm8514a_device::ibm8514_gpstatus_r()
{
    uint16_t ret = 0x0000;

    if (ibm8514.gpbusy == true)
        ret |= 0x0200;
    if (ibm8514.data_avail == true)
        ret |= 0x0100;
    return ret;
}

void ibm8514a_device::ibm8514_draw_vector(uint8_t len, uint8_t dir, bool draw)
{
    uint32_t offset;
    int x = 0;

    while (x <= len)
    {
        offset = (ibm8514.curr_y * IBM8514_LINE_LENGTH) + ibm8514.curr_x;
        if (draw)
            ibm8514_write(offset, offset);
        switch (dir)
        {
        case 0:  // 0 degrees
            ibm8514.curr_x++;
            break;
        case 1:  // 45 degrees
            ibm8514.curr_x++;
            ibm8514.curr_y--;
            break;
        case 2:  // 90 degrees
            ibm8514.curr_y--;
            break;
        case 3:  // 135 degrees
            ibm8514.curr_y--;
            ibm8514.curr_x--;
            break;
        case 4:  // 180 degrees
            ibm8514.curr_x--;
            break;
        case 5:  // 225 degrees
            ibm8514.curr_x--;
            ibm8514.curr_y++;
            break;
        case 6:  // 270 degrees
            ibm8514.curr_y++;
            break;
        case 7:  // 315 degrees
            ibm8514.curr_y++;
            ibm8514.curr_x++;
            break;
        }
        x++;
    }
}

void ibm8514a_device::ibm8514_cmd_w(uint16_t data)
{
    int x, y;
    int pattern_x, pattern_y;
    uint32_t off, src;
    uint8_t readmask;

    ibm8514.current_cmd = data;
    ibm8514.src_x = 0;
    ibm8514.src_y = 0;
    ibm8514.bus_size = (data & 0x0600) >> 9;

    switch (data & 0xe000)
    {
    case 0x0000: 
        ibm8514.state = IBM8514_IDLE;
        ibm8514.gpbusy = false;
        break;

    case 0x2000:  // Line
        ibm8514.state = IBM8514_IDLE;
        ibm8514.gpbusy = false;
        if (data & 0x0008)
        {
            if (data & 0x0100)
            {
                ibm8514.state = IBM8514_DRAWING_LINE;
                ibm8514.data_avail = true;
            }
            else
            {
                ibm8514_draw_vector(ibm8514.rect_width, (data & 0x00e0) >> 5, data & 0x0010);
                ibm8514.state = IBM8514_IDLE;
                ibm8514.gpbusy = false;
            }
        }
        else
        {
            int dx = ibm8514.rect_width;
            int16_t cx = ibm8514.curr_x;
            int16_t cy = ibm8514.curr_y;

            int ex = ibm8514.line_errorterm;
            int ax = ibm8514.line_axial_step;
            int dg = ibm8514.line_diagonal_step;

            for (x = 0; x <= dx; x++)
            {
                off = (cy * IBM8514_LINE_LENGTH) + cx;
                ibm8514.curr_x = cx;
                ibm8514.curr_y = cy;
                if (data & 0x0010)   
                    ibm8514_write(off, off);

                if (ex >= 0)
                {
                    // step along minor axis
                    if (data & 0x0040)
                    {
                        // Y major
                        if (data & 0x0020)
                            cx++;
                        else
                            cx--;
                    }
                    else
                    {
                        // X major
                        if (data & 0x0080)
                            cy++;
                        else
                            cy--;
                    }
                    ex += dg;   // diagonal step
                }
                else
                {
                    ex += ax;   // axial step
                }

                // step along major axis
                if (data & 0x0040)
                {
                    // Y major
                    if (data & 0x0080)
                        cy++;
                    else
                        cy--;
                }
                else
                {
                    // X major
                    if (data & 0x0020)
                        cx++;
                    else
                        cx--;
                }
            }
            ibm8514.curr_x = cx;
            ibm8514.curr_y = cy;
            ibm8514.state = IBM8514_IDLE;
            ibm8514.gpbusy = false;
        }
        break;

    case 0x4000:  // Rectangle Fill
        if (data & 0x0100) 
        {
            ibm8514.state = IBM8514_DRAWING_RECT;
            ibm8514.bus_size = (data & 0x0600) >> 9;
            ibm8514.data_avail = true;
            ibm8514.gpbusy = true;
            LOG("8514/A: Rect Fill WAIT %04x curr=%d,%d rect=%dx%d\n",
                data, ibm8514.curr_x, ibm8514.curr_y,
                ibm8514.rect_width, ibm8514.rect_height);
            break;
        }
        ibm8514.gpbusy = true;
        off = 0;
        off += (IBM8514_LINE_LENGTH * ibm8514.curr_y);
        off += ibm8514.curr_x;
        for (y = 0; y <= ibm8514.rect_height; y++)
        {
            for (x = 0; x <= ibm8514.rect_width; x++)
            {
                if (data & 0x0020)
                    ibm8514_write((off + x) % m_vga->vga.svga_intf.vram_size, (off + x) % m_vga->vga.svga_intf.vram_size);
                else
                    ibm8514_write((off - x) % m_vga->vga.svga_intf.vram_size, (off - x) % m_vga->vga.svga_intf.vram_size);
                if (ibm8514.current_cmd & 0x0020)
                {
                    ibm8514.curr_x++;
                    if (ibm8514.curr_x > ibm8514.prev_x + ibm8514.rect_width)
                    {
                        ibm8514.curr_x = ibm8514.prev_x;
                        ibm8514.src_x = 0;
                        if (ibm8514.current_cmd & 0x0080)
                            ibm8514.curr_y++;
                        else
                            ibm8514.curr_y--;
                    }
                }
                else
                {
                    ibm8514.curr_x--;
                    if (ibm8514.curr_x < ibm8514.prev_x - ibm8514.rect_width)
                    {
                        ibm8514.curr_x = ibm8514.prev_x;
                        ibm8514.src_x = 0;
                        if (ibm8514.current_cmd & 0x0080)
                            ibm8514.curr_y++;
                        else
                            ibm8514.curr_y--;
                    }
                }
            }
            if (data & 0x0080)
                off += IBM8514_LINE_LENGTH;
            else
                off -= IBM8514_LINE_LENGTH;
        }
        ibm8514.state = IBM8514_IDLE;
        ibm8514.gpbusy = false;
        break;

    case 0xc000:  
        ibm8514.gpbusy = true;
        off = 0;
        off += (IBM8514_LINE_LENGTH * ibm8514.dest_y);
        off += ibm8514.dest_x;
        src = 0;
        src += (IBM8514_LINE_LENGTH * ibm8514.curr_y);
        src += ibm8514.curr_x;
        readmask = ((ibm8514.read_mask & 0x01) << 7) | ((ibm8514.read_mask & 0xfe) >> 1);
        for (y = 0; y <= ibm8514.rect_height; y++)
        {
            for (x = 0; x <= ibm8514.rect_width; x++)
            {
                if ((ibm8514.pixel_control & 0xc0) == 0xc0)
                {
                    // video memory determines mix
                    if (data & 0x0020)
                    {
                        if (m_vga->mem_linear_r((src + x)) & ~readmask)
                            ibm8514_write(off + x, src + x);
                    }
                    else
                    {
                        if (m_vga->mem_linear_r((src - x)) & ~readmask)
                            ibm8514_write(off - x, src - x);
                    }
                }
                else
                {
                    if (data & 0x0020)
                        ibm8514_write(off + x, src + x);
                    else
                        ibm8514_write(off - x, src - x);
                }
                if (ibm8514.current_cmd & 0x0020)
                {
                    ibm8514.curr_x++;
                    if (ibm8514.curr_x > ibm8514.prev_x + ibm8514.rect_width)
                    {
                        ibm8514.curr_x = ibm8514.prev_x;
                        ibm8514.src_x = 0;
                        if (ibm8514.current_cmd & 0x0080)
                            ibm8514.curr_y++;
                        else
                            ibm8514.curr_y--;
                    }
                }
                else
                {
                    ibm8514.curr_x--;
                    if (ibm8514.curr_x < ibm8514.prev_x - ibm8514.rect_width)
                    {
                        ibm8514.curr_x = ibm8514.prev_x;
                        ibm8514.src_x = 0;
                        if (ibm8514.current_cmd & 0x0080)
                            ibm8514.curr_y++;
                        else
                            ibm8514.curr_y--;
                    }
                }
            }
            if (data & 0x0080)
            {
                src += IBM8514_LINE_LENGTH;
                off += IBM8514_LINE_LENGTH;
            }
            else
            {
                src -= IBM8514_LINE_LENGTH;
                off -= IBM8514_LINE_LENGTH;
            }
        }
        ibm8514.state = IBM8514_IDLE;
        ibm8514.gpbusy = false;
        ibm8514.curr_x = ibm8514.prev_x;
        ibm8514.curr_y = ibm8514.prev_y;
        break;

    case 0xe000:  // Pattern Fill
        ibm8514.gpbusy = true;
        off = 0;
        off += (IBM8514_LINE_LENGTH * ibm8514.dest_y);
        off += ibm8514.dest_x;
        src = 0;
        src += (IBM8514_LINE_LENGTH * ibm8514.curr_y);
        src += ibm8514.curr_x;
        if (data & 0x0020)
            pattern_x = 0;
        else
            pattern_x = 7;
        if (data & 0x0080)
            pattern_y = 0;
        else
            pattern_y = 7;

        for (y = 0; y <= ibm8514.rect_height; y++)
        {
            for (x = 0; x <= ibm8514.rect_width; x++)
            {
                if (data & 0x0020)
                {
                    ibm8514_write(off + x, src + pattern_x);
                    pattern_x++;
                    if (pattern_x >= 8)
                        pattern_x = 0;
                }
                else
                {
                    ibm8514_write(off - x, src - pattern_x);
                    pattern_x--;
                    if (pattern_x < 0)
                        pattern_x = 7;
                }
            }
            // reset pattern X for each row
            if (data & 0x0020)
                pattern_x = 0;
            else
                pattern_x = 7;

            if (data & 0x0080)
            {
                pattern_y++;
                src += IBM8514_LINE_LENGTH;
                if (pattern_y >= 8)
                {
                    pattern_y = 0;
                    src -= (IBM8514_LINE_LENGTH * 8); 
                }
                off += IBM8514_LINE_LENGTH;
            }
            else
            {
                pattern_y--;
                src -= IBM8514_LINE_LENGTH;
                if (pattern_y < 0)
                {
                    pattern_y = 7;
                    src += (IBM8514_LINE_LENGTH * 8);  
                }
                off -= IBM8514_LINE_LENGTH;
            }
        }
        ibm8514.state = IBM8514_IDLE;
        ibm8514.gpbusy = false;
        break;

    default:
        ibm8514.state = IBM8514_IDLE;
        ibm8514.gpbusy = false;
        LOG("8514/A: Unknown command: %04x\n", data);
        break;
    }
}

uint16_t ibm8514a_device::ibm8514_desty_r()
{
    return ibm8514.line_axial_step;
}

void ibm8514a_device::ibm8514_desty_w(uint16_t data)
{
    ibm8514.line_axial_step = data;
    ibm8514.dest_y = data;
    LOG("8514/A: Line Axial Step / Destination Y write %04x\n", data);
}

uint16_t ibm8514a_device::ibm8514_destx_r()
{
    return ibm8514.line_diagonal_step;
}

void ibm8514a_device::ibm8514_destx_w(uint16_t data)
{
    ibm8514.line_diagonal_step = data;
    ibm8514.dest_x = data;
    LOG("8514/A: Line Diagonal Step / Destination X write %04x\n", data);
}


void ibm8514a_device::ibm8514_draw_ssv(uint8_t data)
{
    uint8_t len = data & 0x0f;
    uint8_t dir = (data & 0xe0) >> 5;
    bool draw = (data & 0x10) ? true : false;

    ibm8514_draw_vector(len, dir, draw);
}

uint16_t ibm8514a_device::ibm8514_ssv_r()
{
    return ibm8514.ssv;
}

void ibm8514a_device::ibm8514_ssv_w(uint16_t data)
{
    ibm8514.ssv = data;

    if (ibm8514.current_cmd & 0x0100)
    {
        ibm8514.state = IBM8514_DRAWING_SSV_1;
        ibm8514.data_avail = true;
        ibm8514.wait_vector_len = ibm8514.ssv & 0x0f;
        ibm8514.wait_vector_dir = (ibm8514.ssv & 0xe0) >> 5;
        ibm8514.wait_vector_draw = (ibm8514.ssv & 0x10) ? true : false;
        ibm8514.wait_vector_count = 0;
        return;
    }

    if (ibm8514.current_cmd & 0x1000)  // byte sequence
    {
        ibm8514_draw_ssv(data & 0xff);
        ibm8514_draw_ssv(data >> 8);
    }
    else
    {
        ibm8514_draw_ssv(data >> 8);
        ibm8514_draw_ssv(data & 0xff);
    }
    LOG("8514/A: Short Stroke Vector write %04x\n", data);
}

void ibm8514a_device::ibm8514_wait_draw_ssv()
{
    uint8_t len = ibm8514.wait_vector_len;
    uint8_t dir = ibm8514.wait_vector_dir;
    bool draw = ibm8514.wait_vector_draw;
    uint8_t count = ibm8514.wait_vector_count;
    uint32_t offset;
    int x;
    int data_size;

    switch (ibm8514.bus_size)
    {
    case 0:  data_size = 8;  break;
    case 1:  data_size = 16; break;
    case 2:  data_size = 32; break;
    default: data_size = 8;  break;
    }

    for (x = 0; x < data_size; x++)
    {
        if (len > count)
        {
            if (ibm8514.state == IBM8514_DRAWING_SSV_1)
            {
                ibm8514.state = IBM8514_DRAWING_SSV_2;
                ibm8514.wait_vector_len = (ibm8514.ssv & 0x0f00) >> 8;
                ibm8514.wait_vector_dir = (ibm8514.ssv & 0xe000) >> 13;
                ibm8514.wait_vector_draw = (ibm8514.ssv & 0x1000) ? true : false;
                ibm8514.wait_vector_count = 0;
                return;
            }
            else if (ibm8514.state == IBM8514_DRAWING_SSV_2)
            {
                ibm8514.state = IBM8514_IDLE;
                ibm8514.gpbusy = false;
                ibm8514.data_avail = false;
                return;
            }
        }

        if (ibm8514.state == IBM8514_DRAWING_SSV_1 || ibm8514.state == IBM8514_DRAWING_SSV_2)
        {
            offset = (ibm8514.curr_y * IBM8514_LINE_LENGTH) + ibm8514.curr_x;
            if (draw)
                ibm8514_write(offset, offset);
            switch (dir)
            {
            case 0: ibm8514.curr_x++;                          break;
            case 1: ibm8514.curr_x++; ibm8514.curr_y--;        break;
            case 2:                    ibm8514.curr_y--;        break;
            case 3: ibm8514.curr_y--; ibm8514.curr_x--;        break;
            case 4: ibm8514.curr_x--;                          break;
            case 5: ibm8514.curr_x--; ibm8514.curr_y++;        break;
            case 6:                    ibm8514.curr_y++;        break;
            case 7: ibm8514.curr_y++; ibm8514.curr_x++;        break;
            }
        }
    }
}

void ibm8514a_device::ibm8514_wait_draw_vector()
{
    uint8_t len = ibm8514.wait_vector_len;
    uint8_t dir = ibm8514.wait_vector_dir;
    bool draw = ibm8514.wait_vector_draw;
    uint8_t count = ibm8514.wait_vector_count;
    uint32_t offset;
    uint8_t data_size = 0;
    int x;

    if (ibm8514.bus_size == 0)  data_size = 8;
    if (ibm8514.bus_size == 1)  data_size = 16;
    if (ibm8514.bus_size == 2)  data_size = 32;

    for (x = 0; x < data_size; x++)
    {
        if (len > count)
        {
            if (ibm8514.state == IBM8514_DRAWING_LINE)
            {
                ibm8514.state = IBM8514_IDLE;
                ibm8514.gpbusy = false;
                ibm8514.data_avail = false;
                return;
            }
        }

        if (ibm8514.state == IBM8514_DRAWING_LINE)
        {
            offset = (ibm8514.curr_y * IBM8514_LINE_LENGTH) + ibm8514.curr_x;
            if (draw)
                ibm8514_write(offset, offset);
            switch (dir)
            {
            case 0: ibm8514.curr_x++;                          break;
            case 1: ibm8514.curr_x++; ibm8514.curr_y--;        break;
            case 2:                    ibm8514.curr_y--;        break;
            case 3: ibm8514.curr_y--; ibm8514.curr_x--;        break;
            case 4: ibm8514.curr_x--;                          break;
            case 5: ibm8514.curr_x--; ibm8514.curr_y++;        break;
            case 6:                    ibm8514.curr_y++;        break;
            case 7: ibm8514.curr_y++; ibm8514.curr_x++;        break;
            }
        }
    }
}


uint16_t ibm8514a_device::ibm8514_currentx_r()
{
    return ibm8514.curr_x;
}

void ibm8514a_device::ibm8514_currentx_w(uint16_t data)
{
    ibm8514.curr_x = data;
    ibm8514.prev_x = data;
    LOG("8514/A: Current X set to %04x (%i)\n", data, ibm8514.curr_x);
}

uint16_t ibm8514a_device::ibm8514_currenty_r()
{
    return ibm8514.curr_y;
}

void ibm8514a_device::ibm8514_currenty_w(uint16_t data)
{
    ibm8514.curr_y = data;
    ibm8514.prev_y = data;
    LOG("8514/A: Current Y set to %04x (%i)\n", data, ibm8514.curr_y);
}


uint16_t ibm8514a_device::ibm8514_width_r()
{
    return ibm8514.rect_width;
}

void ibm8514a_device::ibm8514_width_w(uint16_t data)
{
    ibm8514.rect_width = data & 0x1fff;
    LOG("8514/A: Major Axis Pixel Count / Rectangle Width write %04x\n", data);
}


uint16_t ibm8514a_device::ibm8514_fgcolour_r()
{
    return ibm8514.fgcolour;
}

void ibm8514a_device::ibm8514_fgcolour_w(uint16_t data)
{
    ibm8514.fgcolour = data;
    LOG("8514/A: Foreground Colour write %04x\n", data);
}

uint16_t ibm8514a_device::ibm8514_bgcolour_r()
{
    return ibm8514.bgcolour;
}

void ibm8514a_device::ibm8514_bgcolour_w(uint16_t data)
{
    ibm8514.bgcolour = data;
    LOG("8514/A: Background Colour write %04x\n", data);
}


uint16_t ibm8514a_device::ibm8514_read_mask_r()
{
    return ibm8514.read_mask & 0xffff;
}

void ibm8514a_device::ibm8514_read_mask_w(uint16_t data)
{
    ibm8514.read_mask = (ibm8514.read_mask & 0xffff0000) | data;
    LOG("8514/A: Read Mask (Low) write = %08x\n", ibm8514.read_mask);
}

uint16_t ibm8514a_device::ibm8514_write_mask_r()
{
    return ibm8514.write_mask & 0xffff;
}

void ibm8514a_device::ibm8514_write_mask_w(uint16_t data)
{
    ibm8514.write_mask = (ibm8514.write_mask & 0xffff0000) | data;
    LOG("8514/A: Write Mask (Low) write = %08x\n", ibm8514.write_mask);
}

uint16_t ibm8514a_device::ibm8514_multifunc_r()
{
    switch (ibm8514.multifunc_sel)
    {
    case 0: return ibm8514.rect_height;
    case 1: return ibm8514.scissors_top;
    case 2: return ibm8514.scissors_left;
    case 3: return ibm8514.scissors_bottom;
    case 4: return ibm8514.scissors_right;
    default:
        LOG("8514/A: Unimplemented multifunction register %i selected\n", ibm8514.multifunc_sel);
        return 0xff;
    }
}

void ibm8514a_device::ibm8514_multifunc_w(uint16_t data)
{
    switch (data & 0xf000)
    {
    case 0x0000:
        ibm8514.rect_height = data & 0x0fff;
        LOG("8514/A: Minor Axis Pixel Count / Rectangle Height write %04x\n", data);
        break;
    case 0x1000:
        ibm8514.scissors_top = data & 0x0fff;
        LOG("S3: Scissors Top write %04x\n", data);
        break;
    case 0x2000:
        ibm8514.scissors_left = data & 0x0fff;
        LOG("S3: Scissors Left write %04x\n", data);
        break;
    case 0x3000:
        ibm8514.scissors_bottom = data & 0x0fff;
        LOG("S3: Scissors Bottom write %04x\n", data);
        break;
    case 0x4000:
        ibm8514.scissors_right = data & 0x0fff;
        LOG("S3: Scissors Right write %04x\n", data);
        break;
    case 0xa000:
        ibm8514.pixel_control = data;
        LOG("S3: Pixel control write %04x\n", data);
        break;
    case 0xe000:
        ibm8514.multifunc_misc = data;
        LOG("S3: Multifunction Miscellaneous write %04x\n", data);
        break;
    case 0xf000:
        ibm8514.multifunc_sel = data & 0x000f;
        LOG("S3: Multifunction select write %04x\n", data);
        break;
    default:
        LOG("S3: Unimplemented multifunction register %i write %03x\n", data >> 12, data & 0x0fff);
        break;
    }
}

void ibm8514a_device::ibm8514_wait_draw()
{
    int x, data_size = 8;
    uint32_t off;
    if (ibm8514.bus_size == 0)  data_size = 8;
    if (ibm8514.bus_size == 1)  data_size = 16;
    if (ibm8514.bus_size == 2)  data_size = 32;
    off = 0;
    off += (IBM8514_LINE_LENGTH * ibm8514.curr_y);
    off += ibm8514.curr_x;
    if (ibm8514.current_cmd & 0x02) // "across plane mode"
    {
        for (x = 0; x < data_size; x++)
        {
            ibm8514_write(off, off);
            if (ibm8514.current_cmd & 0x0020)
            {
                off++;
                ibm8514.curr_x++;
                if (ibm8514.curr_x > ibm8514.prev_x + ibm8514.rect_width)
                {
                    ibm8514.curr_x = ibm8514.prev_x;
                    ibm8514.src_x = 0;
                    if (ibm8514.current_cmd & 0x0080)
                    {
                        ibm8514.curr_y++;
                        if (ibm8514.curr_y > ibm8514.prev_y + ibm8514.rect_height)
                        {
                            ibm8514.state = IBM8514_IDLE;
                            ibm8514.data_avail = false;
                            ibm8514.gpbusy = false;
                        }
                    }
                    else
                    {
                        ibm8514.curr_y--;
                        if (ibm8514.curr_y < ibm8514.prev_y - ibm8514.rect_height)
                        {
                            ibm8514.state = IBM8514_IDLE;
                            ibm8514.data_avail = false;
                            ibm8514.gpbusy = false;
                        }
                    }
                    return;
                }
            }
            else
            {
                off--;
                ibm8514.curr_x--;
                if (ibm8514.curr_x < ibm8514.prev_x - ibm8514.rect_width)
                {
                    ibm8514.curr_x = ibm8514.prev_x;
                    ibm8514.src_x = 0;
                    if (ibm8514.current_cmd & 0x0080)
                    {
                        ibm8514.curr_y++;
                        if (ibm8514.curr_y > ibm8514.prev_y + ibm8514.rect_height)
                        {
                            ibm8514.state = IBM8514_IDLE;
                            ibm8514.gpbusy = false;
                            ibm8514.data_avail = false;
                        }
                    }
                    else
                    {
                        ibm8514.curr_y--;
                        if (ibm8514.curr_y < ibm8514.prev_y - ibm8514.rect_height)
                        {
                            ibm8514.state = IBM8514_IDLE;
                            ibm8514.gpbusy = false;
                            ibm8514.data_avail = false;
                        }
                    }
                    return;
                }
            }
        }
    }
    else
    {
        // "through plane" mode (single pixel)
        for (x = 0; x < data_size; x += 8)
        {
            ibm8514_write(off, off);

            if (ibm8514.current_cmd & 0x0020)
            {
                off++;
                ibm8514.curr_x++;
                if (ibm8514.curr_x > ibm8514.prev_x + ibm8514.rect_width)
                {
                    ibm8514.curr_x = ibm8514.prev_x;
                    ibm8514.src_x = 0;
                    if (ibm8514.current_cmd & 0x0080)
                    {
                        ibm8514.curr_y++;
                        if (ibm8514.curr_y > ibm8514.prev_y + ibm8514.rect_height)
                        {
                            ibm8514.state = IBM8514_IDLE;
                            ibm8514.gpbusy = false;
                            ibm8514.data_avail = false;
                        }
                    }
                    else
                    {
                        ibm8514.curr_y--;
                        if (ibm8514.curr_y < ibm8514.prev_y - ibm8514.rect_height)
                        {
                            ibm8514.state = IBM8514_IDLE;
                            ibm8514.gpbusy = false;
                            ibm8514.data_avail = false;
                        }
                    }
                    return;
                }
            }
            else
            {
                off--;
                ibm8514.curr_x--;
                if (ibm8514.curr_x < ibm8514.prev_x - ibm8514.rect_width)
                {
                    ibm8514.curr_x = ibm8514.prev_x;
                    ibm8514.src_x = 0;
                    if (ibm8514.current_cmd & 0x0080)
                    {
                        ibm8514.curr_y++;
                        if (ibm8514.curr_y > ibm8514.prev_y + ibm8514.rect_height)
                        {
                            ibm8514.state = IBM8514_IDLE;
                            ibm8514.gpbusy = false;
                            ibm8514.data_avail = false;
                        }
                    }
                    else
                    {
                        ibm8514.curr_y--;
                        if (ibm8514.curr_y < ibm8514.prev_y - ibm8514.rect_height)
                        {
                            ibm8514.state = IBM8514_IDLE;
                            ibm8514.gpbusy = false;
                            ibm8514.data_avail = false;
                        }
                    }
                    return;
                }
            }
        }
    }
}

uint16_t ibm8514a_device::ibm8514_backmix_r()
{
    return ibm8514.bgmix;
}

void ibm8514a_device::ibm8514_backmix_w(uint16_t data)
{
    ibm8514.bgmix = data;
    LOG("8514/A: BG Mix write %04x\n", data);
}

uint16_t ibm8514a_device::ibm8514_foremix_r()
{
    return ibm8514.fgmix;
}

void ibm8514a_device::ibm8514_foremix_w(uint16_t data)
{
    ibm8514.fgmix = data;
    LOG("8514/A: FG Mix write %04x\n", data);
}

uint16_t ibm8514a_device::ibm8514_pixel_xfer_r(uint32_t offset)
{
    if (offset == 1)
        return (ibm8514.pixel_xfer & 0xffff0000) >> 16;
    else
        return ibm8514.pixel_xfer & 0x0000ffff;
}

void ibm8514a_device::ibm8514_pixel_xfer_w(uint32_t offset, uint16_t data)
{
    if (offset == 1)
        ibm8514.pixel_xfer = (ibm8514.pixel_xfer & 0x0000ffff) | (data << 16);
    else
        ibm8514.pixel_xfer = (ibm8514.pixel_xfer & 0xffff0000) | data;

    if (ibm8514.state == IBM8514_DRAWING_RECT)
        ibm8514_wait_draw();

    if (ibm8514.state == IBM8514_DRAWING_SSV_1 || ibm8514.state == IBM8514_DRAWING_SSV_2)
        ibm8514_wait_draw_ssv();

    if (ibm8514.state == IBM8514_DRAWING_LINE)
        ibm8514_wait_draw_vector();

    LOG("8514/A: Pixel Transfer = %08x\n", ibm8514.pixel_xfer);
}

uint8_t ibm8514a_device::ibm8514_status_r(uint32_t offset)
{
    switch (offset)
    {
    case 0:
        return m_vga->vga_vblank() << 1;
    case 2:
        return 0xff;
    case 3:
    case 4:
    case 5:
        return 0x00;
    }
    return 0;
}

void ibm8514a_device::ibm8514_htotal_w(uint32_t offset, uint8_t data)
{
    switch (offset)
    {
    case 0:
        ibm8514.htotal = data & 0xff;
        break;
    case 2:
    case 3:
    case 4:
    case 5:
        break;
    }
    LOG("8514/A: Horizontal total write %04x\n", data);
}

uint16_t ibm8514a_device::ibm8514_substatus_r()
{
    if (m_vga->vga_vblank() != 0)
        ibm8514.substatus |= 0x01;
    return ibm8514.substatus;
}

void ibm8514a_device::ibm8514_subcontrol_w(uint16_t data)
{
    ibm8514.subctrl = data;
    ibm8514.substatus &= ~(data & 0x0f);  // reset interrupts
}

uint16_t ibm8514a_device::ibm8514_subcontrol_r()
{
    return ibm8514.subctrl;
}

uint16_t ibm8514a_device::ibm8514_htotal_r()
{
    return ibm8514.htotal;
}

uint16_t ibm8514a_device::ibm8514_vtotal_r()
{
    return ibm8514.vtotal;
}

void ibm8514a_device::ibm8514_vtotal_w(uint16_t data)
{
    ibm8514.vtotal = data;
    LOG("8514/A: Vertical total write %04x\n", data);
}

uint16_t ibm8514a_device::ibm8514_vdisp_r()
{
    return ibm8514.vdisp;
}

void ibm8514a_device::ibm8514_vdisp_w(uint16_t data)
{
    ibm8514.vdisp = data;
    LOG("8514/A: Vertical Displayed write %04x\n", data);
}

uint16_t ibm8514a_device::ibm8514_vsync_r()
{
    return ibm8514.vsync;
}

void ibm8514a_device::ibm8514_vsync_w(uint16_t data)
{
    ibm8514.vsync = data;
    LOG("8514/A: Vertical Sync write %04x\n", data);
}

void ibm8514a_device::ibm8514_display_ctrl_w(uint16_t data)
{
    ibm8514.display_ctrl = data & 0x7e;
    switch (data & 0x60)
    {
    case 0x00:
        break;  // do nothing
    case 0x20:
        ibm8514.enabled = true;  // enable 8514/A
        break;
    case 0x40:
    case 0x60:  // reset (does this disable the 8514/A?)
        ibm8514.enabled = false;
        break;
    }
}

void ibm8514a_device::ibm8514_advfunc_w(uint16_t data)
{
    ibm8514.advfunction_ctrl = data;
    ibm8514.passthrough = data & 0x0001;
}

void ibm8514a_device::enabled()
{
    ibm8514.state = IBM8514_IDLE;
    ibm8514.gpbusy = false;
}
