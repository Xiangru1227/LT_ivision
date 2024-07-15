//*************************************************************************************************************************
/// \class api::common::Observer
/// \brief This is a template abstract class that has to be extended to other
/// classes to get the data notifications \details The Observer Pattern defines
/// a one-to-many dependency between objects so that when one object changes
/// state, all of its dependents are notified and updated automatically.
///
/// \class api::common::Publisher
/// \brief This is a template class that has to be extended to other classes to
/// publish the results. Interested clients can register itself as the observer
/// and wait for the update.
//*************************************************************************************************************************
#pragma once

#ifndef PUBLISHER_OBSERVER_H_
#define PUBLISHER_OBSERVER_H_

#include <algorithm>
#include <cstddef>
#include <future>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace api {
namespace common {
template <class T>
class Observer {
 public:
  /// Default constructor
  Observer(){};

  /// Virtual destructor.
  virtual ~Observer(){};

  /// Pure virtual function to get update of the published result.
  /// This function has to be implemented in the class which extends this class.
  /// \param[in] data Data received after publishing
  /// \note This function must be as small as possible such as copying data to
  /// another variable and notifying. Display, or any other blocking or time
  /// consuming functions should not be called in this function as this will
  /// block other operations in publishing thread.
  virtual void Update(const T &data) = 0;

  /// Utility function to tell if 2 observers are same
  bool operator==(const Observer<T> *observer) {
    return (this == observer);
  }
};

template <class T>
class ObserverHashFunction {
 public:
  /// Define the hash function here
  constexpr std::size_t operator()(const Observer<T> *observer) const {
    return reinterpret_cast<size_t>(observer) >> 3;
  }
};

template <class T>
class Publisher {
 public:
  constexpr static int NO_TIMEOUT = -1;

  /// Default constructor
  Publisher(int slow_observer_timeout_ms = NO_TIMEOUT)
      : slow_observer_timeout_ms_(slow_observer_timeout_ms) {
  }

  /// Virtual destructor.
  virtual ~Publisher() {
  }

  /// \brief Function to register as observer for this publisher.
  /// \details This class is marked as final, that means it cannot be overridden
  /// by the extending class. This function checks if the observer is already
  /// registered and if it is not registered it will register again. param[in]
  /// observer Pointer to the class that extends the Observer class
  virtual void RegisterObserver(Observer<T> *observer) final {
    std::lock_guard<std::mutex> lock(observer_mtx_);
    if (observer_map_.find(observer) != observer_map_.end()) {
      return;
    }
    // default-construct an ObserverInfo
    // then set its observer_not_responding to false
    observer_map_[observer].observer_not_responding = false;
  }

  /// \brief Function to de-register the observer
  /// \details This class is marked as final, that means it cannot be overridden
  /// by the extending class. \param[in] observer Pointer to the class that
  /// extends the Observer class
  virtual void RemoveObserver(Observer<T> *observer) final {
    std::lock_guard<std::mutex> lock(observer_mtx_);
    if (observer_map_.find(observer) == observer_map_.end()) {
      return;
    }

    observer_map_.erase(observer);
  }

  /// \brief Function to Notify all of its registered observers
  /// \details This class is marked as final, that means it cannot be overridden
  /// by the extending class. This will check if the observer list has any of
  /// its members. If no members, it will straight away return. \param[in] data
  /// Data to be published to all subscribers
  virtual void NotifyObservers(const T &data) final {
    // check the observer list size, if 0, return
    if (observer_map_.empty()) {
      return;
    }

    // Call the observers update function with task result.
    std::lock_guard<std::mutex> lock(observer_mtx_);
    for (auto it = observer_map_.begin(); it != observer_map_.end(); ++it) {
      if (slow_observer_timeout_ms_ < 0) {
        //No timeout, just block.
        it->first->Update(data);
      } else {
        if (it->second.observer_update_future.valid()) {
          // We tried to notify the observer earlier. Check if it actually responded or not
          if (it->second.observer_not_responding) {
            // Observer is known to be a "slow observer". Don't wait for it, just skip it if it
            // isn't ready
            if (it->second.observer_update_future.wait_for(std::chrono::milliseconds(0)) ==
                std::future_status::ready) {
              // Since the observer was ready this time, it's no longer considered a "slow observer"
              it->second.observer_not_responding = false;
              it->second.observer_update_future = std::async(
                  std::launch::async,
                  [observer = it->first, data_local = data](const T &d) { observer->Update(d); },
                  data);
            }
          } else {
            if (it->second.observer_update_future.wait_for(std::chrono::milliseconds(
                    slow_observer_timeout_ms_)) != std::future_status::ready) {
              // Observer didn't respond in time. Mark it as a slow observer and skip it this
              // iteration
              it->second.observer_not_responding = true;
            } else {
              // Notify the observer
              it->second.observer_update_future = std::async(
                  std::launch::async,
                  [observer = it->first, data_local = data](const T &d) { observer->Update(d); },
                  data);
            }
          }
        } else {
          // First time notifying the observer. Just notify it directly
          it->second.observer_update_future = std::async(
              std::launch::async,
              [observer = it->first, data_local = data](const T &d) { observer->Update(d); }, data);
        }
      }
    }
  }

 private:
  struct ObserverInfo {
    // The future from std::async. Only used to check if the observer finished or not
    std::future<void> observer_update_future;
    // Whether the observer timed out the last time it was notified
    bool observer_not_responding;
  };

  std::mutex observer_mtx_;  ///< Mutex to protect the access of the observer list
  std::unordered_map<Observer<T> *, ObserverInfo, ObserverHashFunction<T>> observer_map_;
  // Timeout in milliseconds before the observer is considered a "slow observer"
  int slow_observer_timeout_ms_;
};
}  // namespace common
}  // namespace api

#endif  // !PUBLISHER_OBSERVER_H_
