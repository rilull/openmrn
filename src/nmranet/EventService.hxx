

#ifndef _NMRANET_GLOBAL_EVENT_HANDLER_
#define _NMRANET_GLOBAL_EVENT_HANDLER_

// This is a workaround for missing shared_ptr.h causing compilation errors. We
// do not use shared_ptr.
#ifndef __CR2_C___4_6_2_BITS_SHARED_PTR_H__
#define __CR2_C___4_6_2_BITS_SHARED_PTR_H__
#endif

#include <memory>

#include "utils/macros.h"
#include "executor/StateFlow.hxx"
#include "nmranet/Defs.hxx"
#include "nmranet/If.hxx"

namespace nmranet
{

class Node;

class EventIteratorFlow;

class EventService : public Service
{
public:
    /** Creates a global event service with no interfaces registered. */
    EventService(ExecutorBase *e);
    /** Creates a global event service that runs on an interface's thread and
     * registers the interface. */
    EventService(If *interface);
    ~EventService();

    /** Registers this global event handler with an interface. This operation
     * will be undone in the destructor. */
    void register_interface(If *interface);

    struct Impl;
    Impl *impl()
    {
        return impl_.get();
    }

    /** Returns true if there are outstanding events that are not yet
     * handled. */
    bool event_processing_pending();

    static EventService *instance;

private:
    std::unique_ptr<Impl> impl_;
};

}; /* namespace nmranet */

#endif // _NMRANET_GLOBAL_EVENT_HANDLER_