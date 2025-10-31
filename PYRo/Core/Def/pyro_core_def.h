#ifndef __PYRO_CORE_DEF_H__
#define __PYRO_CORE_DEF_H__



namespace pyro
{
enum status_t
{
    PYRO_OK      = 0x00,
    PYRO_ERROR   = 0x01,
    PYRO_BUSY    = 0X02,
    PYRO_TIMEOUT = 0X03,
    PYRO_NO_MEMORY   = 0X04,
    PYRO_PARAM_ERROR = 0X05,
    PYRO_NOT_FOUND   = 0X06,
    PYRO_WARNING     = 0X07
};

constexpr float PI=3.14159265358979323846f;

#define CHECK_HAL_RET(ret)        if(HAL_OK != ret)           \
                                  {                           \
                                    return PYRO_ERROR;        \
                                  }                                    

#define CHECK_OS_RET(ret)         if(pdPASS != ret)           \
                                  {                           \
                                    return PYRO_ERROR;        \
                                  }    

#ifdef USE_LOG
#define PYRO_ASSERT_RET(expr)((expr) ? PYRO_OK : log((uint8_t *)__FILE__, __LINE__))
#else
#define PYRO_ASSERT_RET(expr) ((void)0U
#endif
#define PYRO_ASSERT_RET_BLOCK(expr) ((expr) ? PYRO_OK : do{}while(1))

}
                                  
#endif
