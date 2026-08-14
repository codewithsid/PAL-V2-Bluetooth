#pragma once

#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

namespace pal {

enum class MessagePriority : unsigned char {
  RESPONSE = 0,
  NORMAL = 1,
  SPECTRUM = 2,
};

enum class EnqueueResult : unsigned char {
  ENQUEUED,
  REPLACED_SPECTRUM,
  REPLACED_NORMAL,
  DROPPED_INCOMING,
  ALLOCATION_FAILED,
};

struct OwnedTelemetryMessage {
  char *data = nullptr;
  size_t length = 0;  // Includes exactly one trailing newline.
  MessagePriority priority = MessagePriority::NORMAL;
};

// Caller supplies synchronization. This keeps the container usable with a
// FreeRTOS mutex in firmware and std::mutex in host-side concurrency tests.
template <size_t Capacity>
class TelemetryMessageQueue {
 public:
  TelemetryMessageQueue() : count_(0) {}
  TelemetryMessageQueue(const TelemetryMessageQueue &) = delete;
  TelemetryMessageQueue &operator=(const TelemetryMessageQueue &) = delete;

  ~TelemetryMessageQueue() {
    for (size_t i = 0; i < count_; ++i) free(messages_[i].data);
  }

  EnqueueResult enqueueJsonCopy(const char *json, size_t length,
                                MessagePriority priority) {
    if (json == nullptr) return EnqueueResult::ALLOCATION_FAILED;

    // Normalize producer input to exactly one trailing newline.
    while (length > 0 && (json[length - 1] == '\n' || json[length - 1] == '\r')) {
      --length;
    }
    char *copy = static_cast<char *>(malloc(length + 2));
    if (copy == nullptr) return EnqueueResult::ALLOCATION_FAILED;
    memcpy(copy, json, length);
    copy[length] = '\n';
    copy[length + 1] = '\0';

    EnqueueResult result = EnqueueResult::ENQUEUED;
    if (count_ == Capacity) {
      const size_t spectrumIndex = findOldest(MessagePriority::SPECTRUM);
      if (spectrumIndex < count_) {
        removeAndFree(spectrumIndex);
        result = EnqueueResult::REPLACED_SPECTRUM;
      } else if (priority == MessagePriority::RESPONSE) {
        const size_t normalIndex = findOldest(MessagePriority::NORMAL);
        if (normalIndex < count_) {
          removeAndFree(normalIndex);
          result = EnqueueResult::REPLACED_NORMAL;
        } else {
          free(copy);
          return EnqueueResult::DROPPED_INCOMING;
        }
      } else {
        free(copy);
        return EnqueueResult::DROPPED_INCOMING;
      }
    }

    messages_[count_].data = copy;
    messages_[count_].length = length + 1;
    messages_[count_].priority = priority;
    ++count_;
    return result;
  }

  bool dequeue(OwnedTelemetryMessage &message) {
    if (count_ == 0) return false;

    size_t selected = count_;
    for (MessagePriority priority : {MessagePriority::RESPONSE,
                                     MessagePriority::NORMAL,
                                     MessagePriority::SPECTRUM}) {
      selected = findOldest(priority);
      if (selected < count_) break;
    }
    if (selected == count_) return false;

    message = messages_[selected];
    removeWithoutFree(selected);
    return true;
  }

  size_t size() const { return count_; }

 private:
  size_t findOldest(MessagePriority priority) const {
    for (size_t i = 0; i < count_; ++i) {
      if (messages_[i].priority == priority) return i;
    }
    return count_;
  }

  void removeWithoutFree(size_t index) {
    for (size_t i = index + 1; i < count_; ++i) messages_[i - 1] = messages_[i];
    --count_;
    messages_[count_] = OwnedTelemetryMessage{};
  }

  void removeAndFree(size_t index) {
    free(messages_[index].data);
    removeWithoutFree(index);
  }

  OwnedTelemetryMessage messages_[Capacity];
  size_t count_;
};

inline void releaseTelemetryMessage(OwnedTelemetryMessage &message) {
  free(message.data);
  message = OwnedTelemetryMessage{};
}

}  // namespace pal
