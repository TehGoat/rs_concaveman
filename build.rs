extern crate cc;

fn main() {
    cc::Build::new()
        .cpp(true)
        .flag("-std=c++11")
        .file("concaveman.cpp")
        .cpp_link_stdlib("stdc++")
        .compile("concaveman-cpp.so");
}