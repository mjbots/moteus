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
    includes = ["./"],
)
