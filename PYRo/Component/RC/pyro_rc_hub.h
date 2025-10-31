#ifndef __PYRO_RC_HUB_H__
#define __PYRO_RC_HUB_H__

#include "pyro_dr16_rc_drv.h"

namespace pyro
{
class rc_hub_t
{
public:
    rc_hub_t();
    ~rc_hub_t();

    enum which_rc_t
    {
        VTO3 = 0,
        DR16 = 1,
    };


    static rc_drv_t *get_instance(which_rc_t which_rc);


};
}
#endif
