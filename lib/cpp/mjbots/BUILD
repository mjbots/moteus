package(default_visibility = ["//visibility:public"])

filegroup(
    name = "src",
    srcs = [
        "moteus/moteus_multiplex.h",
        "moteus/moteus_optional.h",
        "moteus/moteus_protocol.h",
        "moteus/moteus_tokenizer.h",
        "moteus/moteus_transport.h",
        "moteus/moteus.h",
    ],
)

cc_library(
    name = "moteus",
    hdrs = [":src"],
    includes = ["./", "./moteus"],
)


cc_test(
    name = "test",
    srcs = [
        "moteus/test/moteus_test.cc",
        "moteus/test/moteus_protocol_test.cc",
        "moteus/test/test_main.cc",
    ],
    copts = [
        "-Ilib/cpp",
    ],
    deps = [
        ":moteus",
        "@boost//:test",
    ],
    size = "small",
)
