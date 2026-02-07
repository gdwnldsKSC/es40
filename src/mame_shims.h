#ifndef ES40_MAME_SHIM_H
#define ES40_MAME_SHIM_H

// mame code compat shims
#ifndef popmessage
#define popmessage(...) do { printf(__VA_ARGS__); } while (0)
#endif

struct mame_machine_stub
{
  // MAME - returns true when using debugger to inspect registers,
  // without triggering their actions like ones that reset things on read
  bool side_effects_disabled() const { return false; }
};

struct mame_machine_provider
{
  mame_machine_stub m_machine_stub_;
  mame_machine_stub& machine() { return m_machine_stub_; }
  const mame_machine_stub& machine() const { return m_machine_stub_; }
};

struct nop_callback
{
  template <typename... Args>
  void operator()(Args&&...) const {}
};


// end more mame shims

#endif // ES40_MAME_SHIM_H
