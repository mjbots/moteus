// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Derive macros for the moteus crate.
//!
//! This crate provides the `#[derive(Setters)]` macro for generating
//! builder-style setter methods.

use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, DeriveInput, Data, Fields};

/// Derives builder-style setter methods for struct fields.
///
/// Each field gets a method with the same name that takes a value and
/// returns `Self`, enabling method chaining.
///
/// `Option<T>` fields are automatically detected: the setter takes `T`
/// and wraps the value in `Some()`.
///
/// # Example
///
/// ```ignore
/// use moteus_derive::Setters;
///
/// #[derive(Default, Setters)]
/// struct Config {
///     timeout: u32,
///     retries: Option<u8>,  // Setter takes u8, wraps in Some()
/// }
///
/// let config = Config::default()
///     .timeout(100)
///     .retries(3);  // Automatically wrapped in Some(3)
/// ```
///
/// # Attributes
///
/// - `#[setters(skip)]` - Skip generating a setter for this field
/// - `#[setters(into)]` - Accept `impl Into<T>` instead of `T`
/// - `#[setters(raw)]` - For `Option<T>` fields, take `Option<T>` directly instead of `T`
///
/// ```ignore
/// #[derive(Default, Setters)]
/// struct Options {
///     #[setters(into)]
///     name: String,
///     #[setters(skip)]
///     internal_id: u64,
///     timeout: Option<u32>,       // Auto-detected, setter takes u32
///     #[setters(raw)]
///     override_val: Option<u32>,  // Setter takes Option<u32>
/// }
///
/// let opts = Options::default()
///     .name("test")    // Accepts &str via Into<String>
///     .timeout(100);   // Wraps in Some()
/// ```
#[proc_macro_derive(Setters, attributes(setters))]
pub fn derive_setters(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;
    let generics = &input.generics;
    let (impl_generics, ty_generics, where_clause) = generics.split_for_impl();

    let methods = match &input.data {
        Data::Struct(data) => match &data.fields {
            Fields::Named(fields) => {
                fields.named.iter().filter_map(|f| {
                    let field_name = f.ident.as_ref()?;
                    let field_ty = &f.ty;

                    // Parse field attributes
                    let mut skip = false;
                    let mut use_into = false;
                    let mut use_raw = false;

                    for attr in &f.attrs {
                        if attr.path().is_ident("setters") {
                            let _ = attr.parse_nested_meta(|meta| {
                                if meta.path.is_ident("skip") {
                                    skip = true;
                                } else if meta.path.is_ident("into") {
                                    use_into = true;
                                } else if meta.path.is_ident("raw") {
                                    use_raw = true;
                                }
                                Ok(())
                            });
                        }
                    }

                    if skip {
                        return None;
                    }

                    // Auto-detect Option<T> types (unless raw is specified)
                    let is_option = !use_raw && is_option_type(field_ty);

                    let (param_ty, value_expr) = if is_option {
                        let inner_ty = extract_option_inner_type(field_ty);
                        if use_into {
                            (quote! { impl Into<#inner_ty> }, quote! { Some(value.into()) })
                        } else {
                            (quote! { #inner_ty }, quote! { Some(value) })
                        }
                    } else if use_into {
                        (quote! { impl Into<#field_ty> }, quote! { value.into() })
                    } else {
                        (quote! { #field_ty }, quote! { value })
                    };

                    let method = quote! {
                        pub fn #field_name(mut self, value: #param_ty) -> Self {
                            self.#field_name = #value_expr;
                            self
                        }
                    };

                    Some(method)
                }).collect::<Vec<_>>()
            }
            _ => vec![],
        },
        _ => vec![],
    };

    let expanded = quote! {
        impl #impl_generics #name #ty_generics #where_clause {
            #(#methods)*
        }
    };

    expanded.into()
}

/// Returns true if the type is `Option<T>`.
fn is_option_type(ty: &syn::Type) -> bool {
    if let syn::Type::Path(type_path) = ty {
        if let Some(segment) = type_path.path.segments.last() {
            return segment.ident == "Option";
        }
    }
    false
}

/// Extracts the inner type T from Option<T>.
/// Panics if the type is not Option<T>.
fn extract_option_inner_type(ty: &syn::Type) -> proc_macro2::TokenStream {
    if let syn::Type::Path(type_path) = ty {
        if let Some(segment) = type_path.path.segments.last() {
            if segment.ident == "Option" {
                if let syn::PathArguments::AngleBracketed(args) = &segment.arguments {
                    if let Some(syn::GenericArgument::Type(inner)) = args.args.first() {
                        return quote! { #inner };
                    }
                }
            }
        }
    }
    panic!("extract_option_inner_type called on non-Option type")
}
