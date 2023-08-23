/** \file queue.h
 * Copyright (c) Microsoft Corporation. All rights reserved.
 * Licensed under the MIT License.
 * Kinect For Azure SDK.
 */

#ifndef FRAME_QUEUE_H
#define FRAME_QUEUE_H

#include <k4a/k4atypes.h>
#include <k4ainternal/capture.h>
#include <k4ainternal/common.h>

#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif



/** Default queue depth for 30 FPS.
 */
#define FRAME_QUEUE_DEFAULT_SIZE 10

/** Handle to the queue module.
 *
 * Handles are created with \ref queue_create and closed
 * with \ref queue_destroy.
 * Invalid handles are set to 0.
 */
K4A_DECLARE_HANDLE(frame_queue_t);

typedef void(k4a_capture_release_t)(k4a_capture_t capture);

/** Open a handle to the queue device.
 *
 * \param queue_depth [IN]
 *  The max number of elements the queue can hold. This value is capped at 10,000.
 *
 * \param queue_name [IN]
 *  The name of the queue, used by the logger to generate error messages.
 *
 * \param queue_handle [OUT]
 *  A pointer to write the opened queue handle to
 *
 * \return K4A_RESULT_SUCCEEDED if the device was opened, otherwise K4A_RESULT_FAILED
 *
 * If successful, \ref queue_create will return a queue handle in the queue
 * parameter.
 *
 * When done with the device, close the handle with \ref queue_destroy
 */
k4a_result_t frame_queue_create(uint32_t queue_depth, const char *queue_name, frame_queue_t *queue_handle, k4a_capture_release_t capture_release);

/** Destroys the handle to the queue device.
 *
 * \param queue_handle [in]
 *  A pointer to the handle to destroy
 *
 * When done the queue will have been destroyed and resources remaining in the queue will have been released.
 */
void frame_queue_destroy(frame_queue_t queue_handle);

/** Adds a \ref k4a_capture_t object to the queue.
 *
 * \param queue_handle [in]
 *  A queue handle
 *
 * \param capture_handle [in]
 *  A capture to add to the queue
 *
 * The queue has a fixed size, when that size is been reached the queue will auto drop the oldest element
 * in favor of the one being added with this call.
 */
void frame_queue_push(frame_queue_t queue_handle, k4a_capture_t capture_handle);


/** Adds a \ref k4a_capture_t object to the queue. If the queue is full prior to adding capture_handle, then the capture
 * that needs to be dropped (the oldest) is written to dropped_handle if a location is provided by the caller.
 *
 * \param queue_handle [in]
 *  A queue handle
 *
 * \param capture_handle [in]
 *  A capture to add to the queue
 *
 * \param dropped_handle [out]
 *  A pointer to a location to return a capture that would have been dropped by the queue due to insufficient storage
 *
 * The queue has a fixed size, when that size is been reached a capture needs to be dropped for this API to succeed. In
 * this case that dropped capture will be returned with dropped_handle.
 */
void frame_queue_push_w_dropped(frame_queue_t queue_handle, k4a_capture_t capture_handle);


/** Removes a \ref k4a_capture_t object from the queue.
 *
 * \param queue_handle [in]
 *  A queue handle
 *
 * \param wait_in_ms [in]
 * If the queue has nothing to return, then this wait will be considered. 0 means do not wait at all. A value of N means
 * the function will wait upto N milliseconds for a sample to arrive. If after the time expires and no sample has
 * arrived then NULL is returned. A timeout of K4A_WAIT_INFINITE will wait indefinitely.
 *
 * \param capture_handle [out]
 * The location to write the memory object to that was just popped off the stack
 *
 * returns \ref K4A_WAIT_RESULT_SUCCEEDED if successful, \ref K4A_WAIT_RESULT_TIMEOUT if no data was available in the
 * period specified, \ref K4A_WAIT_RESULT_FAILED if the queue was stopped/destroyed while waiting.
 */
k4a_wait_result_t frame_queue_pop(frame_queue_t queue_handle, int32_t wait_in_ms, k4a_capture_t *capture_handle);

/** Enables the queue for accepting data
 *
 * \param queue_handle [in]
 *  A queue handle
 */
void frame_queue_enable(frame_queue_t queue_handle);

/** Disable the queue for accepting data
 *
 * \param queue_handle [in]
 *  A queue handle
 *
 * When the queue is disabled it also drops the captures remaining in the queue.
 */
void frame_queue_disable(frame_queue_t queue_handle);

/** Notify the queue that it needs to stop
 *
 * \param queue_handle [in]
 *  A queue handle
 *
 * An implicit disable occurs if the queue is stopped
 */
void frame_queue_stop(frame_queue_t queue_handle);

//获取队列的队头元素
k4a_capture_t get_frame_queue_front(frame_queue_t queue_handle);

#ifdef __cplusplus
}
#endif

#endif /* FRAME_QUEUE_H */
