/** \file imusync.h
 * Copyright (c) Microsoft Corporation. All rights reserved.
 * Licensed under the MIT License.
 * Kinect For Azure SDK.
 *
 * Synchronize depth and color captures
 */

#ifndef IMUSYNC_H
#define IMUSYNC_H

//TODO:delete
#include <k4a/k4atypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _imu_frame_data
{
    float data[3];
    uint64_t timestamp;
    float temp;
} imu_frame_data;

typedef struct _imu_sync_frame_data
{
    float accel_data[3];
    float gyro_data[3];
    uint64_t timestamp;
    float temp;
} imu_sync_frame_data;

typedef enum _imu_data_type
{
    ACCEL_FRAME_TYPE = 0,
    GYRO_FRAME_TYPE = 1,
} imu_data_type;

/** Handle to the imusync module
 *
 * Handles are created with imusync_create() and closed
 * with \ref color_destroy.
 * Invalid handles are set to 0.
 */
K4A_DECLARE_HANDLE(imusync_t);

/** Creates an imusync instance
 *
 * \param imusync_handle
 * pointer to a handle location to store the handle. This is only written on K4A_RESULT_SUCCEEDED;
 *
 * To cleanup this resource call capturesync_destroy().
 *
 * \ref K4A_RESULT_SUCCEEDED is returned on success
 */
k4a_result_t imusync_create(imusync_t *imusync_handle);

/** Destroys an capturesync instance
 *
 * \param imusync_handle
 * The capturesync handle to destroy
 *
 * This call only cleans up the capturesync handle.
 * This function should not be called until all outstanding ::k4a_capture_t objects are freed.
 *
 * This function should return 0 to indicate the number of outstanding allocations. Consider this the
 * number of leaked allocations.
 */
void imusync_destroy(imusync_t imusync_handle);

/** Prepares the capturesync object for synchronizing color and depth captures
 *
 * \param imusync_handle
 * The capturesync handle from capturesync_create()
 * \remarks
 * Enables the capturesync to enable its queues and begin synchronizing depth and color frames
 */
k4a_result_t imusync_start(imusync_t imusync_handle);

/** Prepares the capturesync object to stop synchronizing color and depth captures
 *
 * \param imusync_handle
 * The capturesync handle from capturesync_create()
 *
 * \remarks
 * Closes queues and unblocks any waiters waiting for the queue
 */
void imusync_stop(imusync_t imusync_handle);

/** Reads a sample from the synchronized capture queue
 *
 * \param imusync_handle
 * The capturesync handle from capturesync_create()
 *
 * \param capture_handle
 * The location to write the capture handle to
 *
 * \remarks
 * Closes queues and unblocks any waiters waiting for the queue
 */
k4a_wait_result_t imusync_get_frame(imusync_t imusync_handle,
                                      k4a_imu_sample_t *capture_handle,
                                          int32_t timeout_in_ms);

/** Capturesync module asynchronously accepts new captures from color and depth modules through this API.
 *
 * \param imusync_handle
 * The capturesync handle from capturesync_create()
 *
 * \param result
 * The result of the USB opperation providing the sample.
 *
 * \param capture_raw
 * A capture of a single color image, or a capture with upto 2 images; depth and IR.
 *
 * \param color_capture
 * True if the capture contains a single color capture. False is the capture contains upto 2 images; depth and IR.
 *
 * \remarks
 * If ::K4A_SUCCEEDED(result) is true then capture_raw must be valid. If ::K4A_SUCCEEDED(result) is false then
 * capture_raw is optional and may be NULL.
 */
void imusync_push_frame(imusync_t imusync_handle, imu_frame_data imu_data, imu_data_type imu_type);

//函数功能：释放内存
void free_imu_buff(k4a_capture_t capture);

#ifdef __cplusplus
}
#endif

#endif /* CAPTURESYNC_H */
