#include "od.h"

ODObjs_t ODObjs;
static uint16_t ODObjsCount = 0;

typedef struct {
    uint16_t index;
    void *obj;
    uint16_t datasize;
    uint16_t attribute;
    int (*update_func)(void);
} OD_entry_t;

static const OD_entry_t ODList[] = {
    {0x2040, &ODObjs.node_id,                   1, ATTR_ROM | ATTR_RW, NULL},
    {0x2051, &ODObjs.motor_r,                   4, ATTR_ROM | ATTR_RW, NULL},
};

static void dictionary_init(void)
{
    ODObjs.node_id = 1;
    ODObjs.motor_r = 0.5629f;
}

static OD_entry_t *find_entry(uint16_t index)
{
    uint16_t min = 0;
    uint16_t max = ODObjsCount - 1;

    while (min < max) 
    {
        uint16_t cur = (min + max) >> 1;
        OD_entry_t* entry = (OD_entry_t*)&ODList[cur];

        if (index == entry->index) 
        {
            return entry;
        }

        if (index < entry->index) 
        {
            max = (cur > 0) ? (cur - 1) : cur;
        } 
        else 
        {
            min = cur + 1;
        }
    }

    if (min == max) 
    {
        OD_entry_t* entry = (OD_entry_t*)&ODList[min];
        if (index == entry->index) 
        {
            return entry;
        }
    }

    return NULL;
}

void OD_init(void)
{
    ODObjsCount = sizeof(ODList) / sizeof(OD_entry_t);
    dictionary_init();
}

uint16_t OD_read(uint16_t idx, uint16_t *data)
{
    uint16_t cs = CS_ERR;
    memset(data, 0, 2);
    OD_entry_t *entry = find_entry(idx);

    if(entry != NULL && (entry->attribute & ATTR_R))
    {
        switch(entry->datasize)
        {
            case 1:
            {
                data[0] = __byte(entry->obj, 0);
                cs = CS_R_ACK_1;
                break;
            }
            case 2:
            {
                data[0] = (__byte(entry->obj, 1) << 8) | __byte(entry->obj, 0);
                cs = CS_R_ACK_2;
                break;
            }
            case 3:
            {
                data[0] = (__byte(entry->obj, 1) << 8) | __byte(entry->obj, 0);
                data[1] = __byte(entry->obj, 2);
                cs = CS_R_ACK_3;
                break;
            }
            case 4:
            {
                data[0] = (__byte(entry->obj, 1) << 8) | __byte(entry->obj, 0);
                data[1] = (__byte(entry->obj, 3) << 8) | __byte(entry->obj, 2);
                cs = CS_R_ACK_4;
                break;
            }
        }
    }
    return cs;
}

uint16_t OD_write_1(uint16_t idx, uint16_t *data)
{
    uint16_t cs = CS_ERR;
    OD_entry_t *entry = find_entry(idx);
    
    if((entry != NULL) && (entry->attribute & ATTR_W) && (entry->datasize == 1))
    {
        if(__byte(entry->obj, 0) != __byte(data, 0))
        {
            __byte(entry->obj, 0) = __byte(data, 0);
            if(entry->attribute & ATTR_ROM)
            {
                load_ram_item_to_eeprom_from_key(0);
                cs = CS_W_ACK;
            }
            else
            {
                cs = CS_W_ACK;
            }
        }
        else
        {
            cs = CS_W_ACK;
        }
    }

    if(cs == CS_W_ACK && entry->update_func != NULL) 
    {
        if(0 != entry->update_func())
        {
            cs = CS_ERR;
        }
    }

    memset(data, 0, 2);

    return cs;
}

uint16_t OD_write_2(uint16_t idx, uint16_t *data)
{
    uint16_t cs = CS_ERR;
    OD_entry_t *entry = find_entry(idx);
    
    if((entry != NULL) && (entry->attribute & ATTR_W) && (entry->datasize == 2))
    {
        if(*(uint16_t *)entry->obj != *(uint16_t *)data)
        {
            *(uint16_t *)entry->obj = *(uint16_t *)data;
            if(entry->attribute & ATTR_ROM)
            {
                load_ram_item_to_eeprom_from_key(0);
                cs = CS_W_ACK;
            }
            else
            {
                cs = CS_W_ACK;
            }
        }
        else
        {
            cs = CS_W_ACK;
        }
    }
    
    if(cs == CS_W_ACK && entry->update_func != NULL) 
    {
        if(0 != entry->update_func())
        {
            cs = CS_ERR;
        }
    }

    memset(data, 0, 2);

    return cs;
}

uint16_t OD_write_4(uint16_t idx, uint16_t *data)
{
    uint16_t cs = CS_ERR;
    OD_entry_t *entry = find_entry(idx);
    
    if((entry != NULL) && (entry->attribute & ATTR_W) && (entry->datasize == 4))
    {
        if(*(uint32_t *)entry->obj != *(uint32_t *)data)
        {
            *(uint32_t *)entry->obj = *(uint32_t *)data;
            if(entry->attribute & ATTR_ROM)
            {
                load_ram_item_to_eeprom_from_key(0);
                cs = CS_W_ACK;
            }
            else
            {
                cs = CS_W_ACK;
            }
        }
        else
        {
            cs = CS_W_ACK;
        }
    }
    
    if(cs == CS_W_ACK && entry->update_func != NULL) 
    {
        if(0 != entry->update_func())
        {
            cs = CS_ERR;
        }
    }

    memset(data, 0, 2);

    return cs;
}
