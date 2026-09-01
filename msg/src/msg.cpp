#include "msg.hpp"

#include <cstring>

namespace msg
{
namespace
{

bool bus_ready = false;

ULONG mutex_wait_option(const publish_opts& opts) noexcept
{
    return opts.from_isr || !opts.block ? TX_NO_WAIT : TX_WAIT_FOREVER;
}

bool subscriber_valid(const subscriber& sub) noexcept
{
    return sub.valid() && sub.layout->initialized &&
           sub.slot_index < max_subscribers_per_topic;
}

} // namespace

types::status init() noexcept
{
    bus_ready = true;
    return types::status::ok;
}

namespace detail
{

types::status initialize_topic(topic_layout& layout, const char* debug_name) noexcept
{
    if (debug_name == nullptr || debug_name[0] == '\0' || layout.payload_bytes == nullptr ||
        layout.payload_size == 0U || layout.payload_alignment == 0U)
    {
        return types::status::invalid_arg;
    }
    if (init() != types::status::ok)
    {
        return types::status::error;
    }

    layout.debug_name = debug_name;
    layout.subscriber_count = 0U;
    for (auto& slot : layout.subscribers)
    {
        slot = {};
    }

    if (tx_mutex_create(&layout.lock, const_cast<CHAR*>(debug_name), TX_NO_INHERIT) != TX_SUCCESS)
    {
        layout.debug_name = nullptr;
        return types::status::error;
    }

    layout.initialized = true;
    return types::status::ok;
}

subscriber subscribe(topic_layout& layout) noexcept
{
    if (!bus_ready || !layout.initialized ||
        tx_mutex_get(&layout.lock, TX_WAIT_FOREVER) != TX_SUCCESS)
    {
        return subscriber::invalid();
    }

    subscriber result = subscriber::invalid();
    for (std::uint8_t index = 0U; index < max_subscribers_per_topic; ++index)
    {
        auto& slot = layout.subscribers[index];
        if (!slot.active)
        {
            slot.active = true;
            slot.pending = false;
            ++layout.subscriber_count;
            result = {&layout, index};
            break;
        }
    }

    tx_mutex_put(&layout.lock);
    return result;
}

types::status publish(topic_layout& layout, const void* data, std::size_t size,
                      publish_opts opts) noexcept
{
    if (!bus_ready || !layout.initialized || data == nullptr || size != layout.payload_size)
    {
        return types::status::invalid_arg;
    }
    if (tx_mutex_get(&layout.lock, mutex_wait_option(opts)) != TX_SUCCESS)
    {
        return types::status::busy;
    }

    std::memcpy(layout.payload_bytes, data, size);
    for (auto& slot : layout.subscribers)
    {
        if (slot.active)
        {
            slot.pending = true;
        }
    }

    tx_mutex_put(&layout.lock);
    return types::status::ok;
}

} // namespace detail

bool available(subscriber sub) noexcept
{
    if (!bus_ready || !subscriber_valid(sub))
    {
        return false;
    }
    const auto& slot = sub.layout->subscribers[sub.slot_index];
    return slot.active && slot.pending;
}

types::status read(subscriber sub, void* buffer, std::size_t size) noexcept
{
    if (!bus_ready || !subscriber_valid(sub) || buffer == nullptr ||
        size != sub.layout->payload_size)
    {
        return types::status::invalid_arg;
    }

    auto& layout = *sub.layout;
    if (tx_mutex_get(&layout.lock, TX_WAIT_FOREVER) != TX_SUCCESS)
    {
        return types::status::error;
    }

    auto& slot = layout.subscribers[sub.slot_index];
    if (!slot.active || !slot.pending)
    {
        tx_mutex_put(&layout.lock);
        return types::status::empty;
    }

    std::memcpy(buffer, layout.payload_bytes, size);
    slot.pending = false;
    tx_mutex_put(&layout.lock);
    return types::status::ok;
}

} // namespace msg
