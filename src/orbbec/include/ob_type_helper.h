#pragma once

#include "k4a/k4a.h"
#include <libobsensor/h/Context.h>
#include <libobsensor/h/Error.h>

#ifdef __cplusplus
extern "C" {
#endif

k4a_result_t check_ob_error(ob_error *error);

#define CHECK_OB_ERROR(ob_err) check_ob_error(ob_err)
#define OB_SUCCEEDED(error) (K4A_RESULT_SUCCEEDED == check_ob_error(error))
#define OB_FAILED(error) (K4A_RESULT_FAILED == check_ob_error(error))

#define CHECK_OB_ERROR_CONTINUE(error)                                                                                 \
    if (K4A_RESULT_FAILED == check_ob_error(error))                                                                    \
    {                                                                                                                  \
        continue;                                                                                                      \
    }

#define CHECK_OB_ERROR_BREAK(error)                                                                                    \
    if (K4A_RESULT_FAILED == check_ob_error(error))                                                                    \
    {                                                                                                                  \
        break;                                                                                                         \
    }

#define CHECK_OB_ERROR_RETURN(error)                                                                                   \
    if (K4A_RESULT_FAILED == check_ob_error(error))                                                                    \
    {                                                                                                                  \
        return;                                                                                                        \
    }

#define CHECK_OB_ERROR_RETURN_K4A_RESULT(ob_err)                                                                       \
    if (K4A_RESULT_FAILED == check_ob_error(ob_err))                                                                   \
    {                                                                                                                  \
        return K4A_RESULT_FAILED;                                                                                      \
    }

#define CHECK_OB_ERROR_RETURN_VALUE(ob_err, value)                                                                     \
    if (K4A_RESULT_FAILED == check_ob_error(ob_err))                                                                   \
    {                                                                                                                  \
        return value;                                                                                                  \
    }

ob_context *get_ob_context_instance();

void get_depthengine_context_instance();

#ifdef __cplusplus
}
#endif