#ifndef THREADPOOL_H
#define THREADPOOL_H

#include <vector>
#include <queue>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <atomic>

class ThreadPool {
public:
    // Constructor: Creates the thread pool with a specified number of threads
    ThreadPool(size_t numThreads);

    // Destructor: Joins all threads
    ~ThreadPool();

    // Submit a task to the pool
    template<class F>
    void enqueue(F&& f);

    // Start processing tasks continuously
    void start();

    // Stop processing tasks
    void stop();

private:
    // Worker function to be run by each thread
    void worker();

    std::vector<std::thread> workers;                        // Vector of worker threads
    std::queue<std::function<void()>> tasks;                 // Queue of tasks
    
    std::mutex queueMutex;                                  // Mutex for task queue
    std::condition_variable condition;                      // Condition variable for task notifications
    std::atomic<bool> stopFlag;                             // Flag to signal stopping the pool
    std::atomic<bool> runningFlag;                          // Flag to signal if the pool is running
};

// Implementation

// Constructor: Create the thread pool and start worker threads
ThreadPool::ThreadPool(size_t numThreads) : stopFlag(false), runningFlag(false) {
    for (size_t i = 0; i < numThreads; ++i) {
        workers.emplace_back(&ThreadPool::worker, this);
    }
}

// Destructor: Join all threads
ThreadPool::~ThreadPool() {
    stopFlag = true;
    condition.notify_all();
    for (std::thread &worker : workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

// Function to enqueue tasks
template<class F>
void ThreadPool::enqueue(F&& f) {
    {
        std::unique_lock<std::mutex> lock(queueMutex);
        tasks.emplace(std::forward<F>(f));
    }
    condition.notify_one();
}

// Worker function: Continuously process tasks
void ThreadPool::worker() {
    while (true) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            condition.wait(lock, [this] {
                return stopFlag || !tasks.empty();
            });
            if (stopFlag && tasks.empty()) {
                return;
            }
            task = std::move(tasks.front());
            tasks.pop();
        }
        task();
        // Check if we should stop processing
        if (stopFlag && tasks.empty()) {
            return;
        }
    }
}

// Start processing tasks continuously
void ThreadPool::start() {
    runningFlag = true;
}

// Stop processing tasks
void ThreadPool::stop() {
    runningFlag = false;
    stopFlag = true;
    condition.notify_all();
}

#endif // THREADPOOL_H
