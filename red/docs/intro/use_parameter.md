## set/get paramter

> [!note]
> 参考 aimrt 可以复用这里 set/get parameter 的方式，来取代 quadruped 中的相关类？

AimRT 中提供了一个简单的模块级 Key-Val 参数功能，模块可以通过调用`CoreRef`句柄的`GetParameterHandle()`接口，获取`aimrt::parameter::ParameterHandleRef`句柄，来使用此功能。该句柄提供的核心接口如下：

```cpp
namespace aimrt::parameter {

class ParameterHandleRef {
 public:
  std::string GetParameter(std::string_view key) const;

  void SetParameter(std::string_view key, std::string_view val) const;
};

}  // namespace aimrt::parameter
```

## log

> [!note]
> 可以复用 aimrt 中的 util/log_util.h 为指定的功能包来提供 log
> 需要依赖 libfmt 或者使用 c++-20 std::format
