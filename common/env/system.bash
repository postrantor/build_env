# use `cd ...` equal `cd ../..`
# only for bash
if [ -n "$BASH_VERSION" ]; then
  function cd() {
    if [[ $1 =~ ^\.{2,}$ ]]; then
      local count=${#1}
      local path=""
      for ((i = 1; i < count; i++)); do
        path+="../"
      done
      builtin cd "$path"
    else
      builtin cd "$@"
    fi
  }
fi
