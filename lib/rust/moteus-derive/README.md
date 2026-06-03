# moteus-derive

Derive macros used by the [`moteus`](https://crates.io/crates/moteus) and
[`moteus-protocol`](https://crates.io/crates/moteus-protocol) crates.

This is an internal support crate for the moteus client library. It is not
intended for direct use, and its API may change to suit the needs of the
moteus crates.

## `#[derive(Setters)]`

Generates builder-style setter methods for struct fields. Each field gets
a method of the same name that sets the field and returns `Self`, enabling
method chaining. `Option<T>` fields are detected automatically: their
setters take a `T` and wrap it in `Some`.

```rust
use moteus_derive::Setters;

#[derive(Default, Setters)]
struct Config {
    timeout: u32,
    retries: Option<u8>,
}

let config = Config::default()
    .timeout(100)
    .retries(3); // wrapped in Some(3) automatically

assert_eq!(config.timeout, 100);
assert_eq!(config.retries, Some(3));
```

### Field Attributes

- `#[setters(skip)]`: do not generate a setter for this field.
- `#[setters(into)]`: the setter accepts `impl Into<T>` instead of `T`.
- `#[setters(raw)]`: for `Option<T>` fields, the setter takes the
  `Option<T>` directly instead of wrapping a `T`.

```rust
use moteus_derive::Setters;

#[derive(Default, Setters)]
struct Options {
    #[setters(into)]
    name: String, // accepts &str
    #[setters(skip)]
    internal_id: u64, // no setter generated
    timeout: Option<u32>, // setter takes u32
    #[setters(raw)]
    override_val: Option<u32>, // setter takes Option<u32>
}

let opts = Options::default()
    .name("test")
    .timeout(100)
    .override_val(Some(5));

assert_eq!(opts.name, "test");
assert_eq!(opts.timeout, Some(100));
assert_eq!(opts.override_val, Some(5));
```

## Building with Bazel

When building from the [moteus repository](https://github.com/mjbots/moteus):

```bash
tools/bazel build //lib/rust/moteus-derive
```
