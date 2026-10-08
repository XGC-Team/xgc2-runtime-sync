# Native validation — 2026-10-09

Scope: isolated source/native runtime only. No live station, APT publication,
vehicle hardware or weak-network end-to-end claim.

SDK: owning common/xrpc work tree rooted at HEAD
`b80f5c8af0621c240634dc294c7c5d1bb8724403`, including then-uncommitted C++20
HTTP/runtime-policy implementation. Tests linked the installed snapshot
`/tmp/sol6-sync-sdk-v2-prefix`, not an assumed published SDK version.

| Installed input | SHA256 |
| --- | --- |
| libxgc2_xrpc_http.so.0.1.0 | 31543fe7819821e13e7e7e4e77f51de4b9a26a191ab6ffcb4332108062fc82b8 |
| libxgc2_xrpc_policy.so.0.1.0 | cb4339995f5be8a15799537f8df9c1cf94d75605cbb90a4a4c2c1860fcb860e5 |
| http.hpp | b14983e71c9d5ee2dd17d0b44ebe3da8a6199a828efb3a4c36e381f76204382c |
| runtime_policy.hpp | 171f3cfd857ac4c59f147184d176f25da8210f09a9b628135d08716a57ec2df7 |

Toolchain: Ubuntu Noble amd64 GCC 13.3, system Boost 1.83, installed native
Noetic libraries. Noetic's CMake export referenced absent liblog4cxx-dev
headers/linker symlink. Official Noble `liblog4cxx-dev 1.1.0-1build3` was
unpacked under `/tmp/sol6-ros-dependency`; a test-only relocated rosconsole
CMake export uses those headers and the existing native `liblog4cxx.so.15`.
No system package, live ROS graph or checked-out SDK source was changed.

Native package configurations passed `-DXgcXrpc_DIR=/tmp/sol6-sync-sdk-v2-prefix/lib/cmake/XgcXrpc`,
`-Drosconsole_DIR=/tmp/sol6-ros-dependency/rosconsole-cmake`,
`-DPYTHON_EXECUTABLE=/usr/bin/python3` and `-DCATKIN_ENABLE_TESTING=ON`.
The product's deployment metadata still targets Focal; these Noble builds do
not establish a Focal GCC9/C++20 or cross-suite ABI release.

Product migration baseline: `7a6ec98f3940620b2e04a88df4e4bad19794fd7e`.

`cmake --build /tmp/sol6-runtime-build -j2` and target
`runtime_node_smoke_test` passed. After sourcing its devel setup,
`bash tests/xrpc_native_smoke.sh` passed 1 real ROS product test (0 errors,
0 failures). Output: `/tmp/sol6-runtime-native-rostest.log`.
The test creates a dedicated rostest master and grants fresh socket/ROS log
allocations. It verifies real cycle/sample/health data, absent old services,
strict schemas, wrong revisions, stop effects, ID replay/reuse, pending
start versus native RUNNING, held observations, sticky terminal receipts and
effective policy sources. Mock clock was selected; production chrony latency
is not measured. C++ host threads/FD/RSS soak and crash/weak-network matrix are
still required before release.

Packaging follow-up: Debian revision incremented for the unpublished source candidate;
package control now declares formal libxgc2-xrpc1>=0.1.0 and Boost JSON>=1.75.
No package was published or installed. The release-set snapshot is unchanged.
Python SDK consumers use an explicitly selected interpreter>=3.10 with the
formal wheel, not a made-up Python SDK Debian package or replaced system Python.
This product exposes native C++ control; it adds no production Python SDK
consumer. Formal Python tooling, when used, requires the selected interpreter.
Focal controlled runtime/toolchain and that selected Python environment remain
unverified deployment gates. No new host Gazebo/GDB fixture was launched after
the user hard stop.
