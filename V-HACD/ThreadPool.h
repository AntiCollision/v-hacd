#pragma once
//********************************************************************************************************************
// Definition of the ThreadPool
//********************************************************************************************************************

class ThreadPool {
public:
    ThreadPool();
    ThreadPool(int worker);
    ~ThreadPool();
    template<typename F, typename... Args>
    auto enqueue(F&& f, Args&& ... args)
#ifndef __cpp_lib_is_invocable
        ->std::future< typename std::result_of< F(Args...) >::type>;
#else
        ->std::future< typename std::invoke_result_t<F, Args...>>;
#endif
private:
    std::vector<std::thread> workers;
    std::deque<std::function<void()>> tasks;
    std::mutex task_mutex;
    std::condition_variable cv;
    bool closed;
    int count;
};

ThreadPool::ThreadPool()
    : ThreadPool(1)
{
}

ThreadPool::ThreadPool(int worker)
    : closed(false)
    , count(0)
{
    workers.reserve(worker);
    for (int i = 0; i < worker; i++)
    {
        workers.emplace_back(
            [this]
            {
                std::unique_lock<std::mutex> lock(this->task_mutex);
            while (true)
            {
                while (this->tasks.empty())
                {
                    if (this->closed)
                    {
                        return;
                    }
                    this->cv.wait(lock);
                }
                auto task = this->tasks.front();
                this->tasks.pop_front();
                lock.unlock();
                task();
                lock.lock();
            }
            }
            );
    }
}

template<typename F, typename... Args>
auto ThreadPool::enqueue(F&& f, Args&& ... args)
#ifndef __cpp_lib_is_invocable
-> std::future< typename std::result_of< F(Args...) >::type>
#else
-> std::future< typename std::invoke_result_t<F, Args...>>
#endif
{

#ifndef __cpp_lib_is_invocable
    using return_type = typename std::result_of< F(Args...) >::type;
#else
    using return_type = typename std::invoke_result_t< F, Args... >;
#endif
    auto task = std::make_shared<std::packaged_task<return_type()> >(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
    auto result = task->get_future();

    {
        std::unique_lock<std::mutex> lock(task_mutex);
        if (!closed)
        {
            tasks.emplace_back([task]
                {
                    (*task)();
                });
            cv.notify_one();
        }
    }

    return result;
}

ThreadPool::~ThreadPool() {
    {
        std::unique_lock<std::mutex> lock(task_mutex);
        closed = true;
    }
    cv.notify_all();
    for (auto&& worker : workers)
    {
        worker.join();
    }
}