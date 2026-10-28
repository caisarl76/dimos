window.BENCHMARK_DATA = {
  "lastUpdate": 1793154170015,
  "repoUrl": "https://github.com/caisarl76/dimos",
  "entries": {
    "go2 replay realtime (arm64)": [
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "51e97981bf339c029b8c1e179c845f714782185e",
          "message": "Use Otava for benchmark analysis (#4298)",
          "timestamp": "2026-10-02T16:57:32+01:00",
          "tree_id": "e7189f416bef7dd6b0820649feab088e70b6afb6",
          "url": "https://github.com/dimensionalOS/dimos/commit/51e97981bf339c029b8c1e179c845f714782185e"
        },
        "date": 1790956831103,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2027.047,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 394,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2737.508,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.352,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.259,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "bogwi@tutamail.com",
            "name": "Dan Vi",
            "username": "bogwi"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d05c743aa98c7ebfec14988c7f0fec216e908215",
          "message": "fix mem concurrency (#4405)",
          "timestamp": "2026-10-02T16:22:10Z",
          "tree_id": "9437fca6ac8ba9597fa9ba6632a322a52d772f65",
          "url": "https://github.com/dimensionalOS/dimos/commit/d05c743aa98c7ebfec14988c7f0fec216e908215"
        },
        "date": 1790958305535,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2027.203,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 399,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2734.559,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.191,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.326,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d5b47cb56cf0af86d9ffe6de5b6a09fc57cc9fbe",
          "message": "fix: bump zenoh crate to 1.10.1 (#4386)",
          "timestamp": "2026-10-02T16:53:44Z",
          "tree_id": "b861fc4bb0ea22884dd9d370e7eeb9076e10efa0",
          "url": "https://github.com/dimensionalOS/dimos/commit/d5b47cb56cf0af86d9ffe6de5b6a09fc57cc9fbe"
        },
        "date": 1790960208283,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2000.301,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 403,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2737.952,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.176,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.23,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e06d97554f54977b80ff0304f4e68a1eb8e5d79d",
          "message": "feat(agent): add planner-based go_to(x,y) skill (#4380)",
          "timestamp": "2026-10-02T10:08:08-07:00",
          "tree_id": "ffafcf0f77777fb795961dd3a4c0201cc30729bc",
          "url": "https://github.com/dimensionalOS/dimos/commit/e06d97554f54977b80ff0304f4e68a1eb8e5d79d"
        },
        "date": 1790961069377,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2025.395,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 402,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.812,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.18,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.652,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5677b6d46c1e3c15a445793586afa1da39c88892",
          "message": "feat(control): operator hold task (#4402)",
          "timestamp": "2026-10-02T19:22:48Z",
          "tree_id": "d20d0cf28fb0c1b75ee67b18398a8f1f54b275db",
          "url": "https://github.com/dimensionalOS/dimos/commit/5677b6d46c1e3c15a445793586afa1da39c88892"
        },
        "date": 1790969138260,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2033.699,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 404,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.586,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.184,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.32,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "943ce13c67e679e75452557f9457f1ba58541091",
          "message": "feat(memory): record explicit JSON streams with source timestamps (#4392)",
          "timestamp": "2026-10-02T15:12:04-07:00",
          "tree_id": "5a1d5b3f6095c1b6ceb917e361f488e4cb42a640",
          "url": "https://github.com/dimensionalOS/dimos/commit/943ce13c67e679e75452557f9457f1ba58541091"
        },
        "date": 1790979300466,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2035.031,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 397,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.585,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.184,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.136,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e9a1ec0498a203187c6ea3e782fba83cb5779535",
          "message": "refactor(manipulation): skills report facts instead of error codes (#4401)",
          "timestamp": "2026-10-03T00:16:58-07:00",
          "tree_id": "258cd2a97575f3575fa0e5a9bda1dca0d5af4937",
          "url": "https://github.com/dimensionalOS/dimos/commit/e9a1ec0498a203187c6ea3e782fba83cb5779535"
        },
        "date": 1791011987648,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2035.688,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 400,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2739.751,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.43,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.652,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "60aee3c05e86ae15c9e38df960918414cf1904c1",
          "message": "r1pro: head cameras straight off V4L2, intrinsics from the factory calibration (#4388)\n\nCo-authored-by: Mustafa <mustafa@dimensionalos.com>",
          "timestamp": "2026-10-03T00:45:21-07:00",
          "tree_id": "5406c0e83bffd810cece15c9bba0687315ae65b0",
          "url": "https://github.com/dimensionalOS/dimos/commit/60aee3c05e86ae15c9e38df960918414cf1904c1"
        },
        "date": 1791013699144,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2024.738,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 403,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.595,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.16,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.151,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "63036454+ruthwikdasyam@users.noreply.github.com",
            "name": "ruthwikdasyam",
            "username": "ruthwikdasyam"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "db3d0ca9f31d726bfea3a78e8a57662292372d50",
          "message": "fix(sim): reject orphaned MuJoCo shared-memory buffers (#4337)",
          "timestamp": "2026-10-03T17:51:09-07:00",
          "tree_id": "91e640f27eba6853ab057c8c66ad6a529bf5263a",
          "url": "https://github.com/dimensionalOS/dimos/commit/db3d0ca9f31d726bfea3a78e8a57662292372d50"
        },
        "date": 1791075237313,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2016.594,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 392,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2737.46,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.141,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.056,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c2189a6ea5f431bf6c9bd2111acc0914d375121d",
          "message": "r1pro: hardware-sync the two head cameras (FSYNC) (#4383)\n\nCo-authored-by: Mustafa <mustafa@dimensionalos.com>",
          "timestamp": "2026-10-05T12:27:10-07:00",
          "tree_id": "ce77c018b749a7b395221b4a3e0c2a7aa0c9a497",
          "url": "https://github.com/dimensionalOS/dimos/commit/c2189a6ea5f431bf6c9bd2111acc0914d375121d"
        },
        "date": 1791229214708,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2038.367,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 398,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.633,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.371,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 148.971,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a79ad67c3f1a9f2a1c9ee73e2a1caf89f67f10e7",
          "message": "perf(sim): faster MuJoCo simulator start-up (#4281)",
          "timestamp": "2026-10-05T22:01:59Z",
          "tree_id": "0965d78653ea1b82a6aee35346f006ad06d7917e",
          "url": "https://github.com/dimensionalOS/dimos/commit/a79ad67c3f1a9f2a1c9ee73e2a1caf89f67f10e7"
        },
        "date": 1791237909318,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2012.555,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 400,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.583,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.184,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.202,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "875e64d284b9bd272f30762e07ccf6e51bc93c10",
          "message": "ci: reject AI co-authors in incoming commits (#4346)",
          "timestamp": "2026-10-05T22:38:50Z",
          "tree_id": "e938dd2ad418f3c2d12a0a32853f7aaf2e3ba2c7",
          "url": "https://github.com/dimensionalOS/dimos/commit/875e64d284b9bd272f30762e07ccf6e51bc93c10"
        },
        "date": 1791240107437,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2025.445,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 399,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2734.453,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.195,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.537,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b18930752de93bbf71c7741c5a1eb8a2d7e05c84",
          "message": "Make Mac tests required (#4321)",
          "timestamp": "2026-10-06T02:01:29+03:00",
          "tree_id": "04f066ab304939e550fcf8e28f748cb9167b81f9",
          "url": "https://github.com/dimensionalOS/dimos/commit/b18930752de93bbf71c7741c5a1eb8a2d7e05c84"
        },
        "date": 1791241468632,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2023.961,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 395,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2735.823,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.18,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.227,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ee0e99f0d5995da8e3a9f2b98355f6bf2ac9c17c",
          "message": "Drop cooldown from internal docker (#4309)",
          "timestamp": "2026-10-06T02:05:25+03:00",
          "tree_id": "9289cbd0bb1f5f06eed8b50355580928979dc118",
          "url": "https://github.com/dimensionalOS/dimos/commit/ee0e99f0d5995da8e3a9f2b98355f6bf2ac9c17c"
        },
        "date": 1791241700044,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2056.336,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 404,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.079,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.164,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.202,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f10c2f3823d485cc9e44ed7c631e0ef75360773d",
          "message": "fix: deskewed pointlio point clouds (#4423)",
          "timestamp": "2026-10-06T02:07:57+03:00",
          "tree_id": "53bd1c32d2569b2210d942bdbf07a21916713c4d",
          "url": "https://github.com/dimensionalOS/dimos/commit/f10c2f3823d485cc9e44ed7c631e0ef75360773d"
        },
        "date": 1791241846917,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2020.012,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 398,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.964,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.18,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.52,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c419f9933087ce5e11140ca3763eff562e20b39a",
          "message": "fix: remove default lidar ip (#4416)",
          "timestamp": "2026-10-06T02:23:05+03:00",
          "tree_id": "995d2db4bd495ec5c467d266969d73aa3dddf01a",
          "url": "https://github.com/dimensionalOS/dimos/commit/c419f9933087ce5e11140ca3763eff562e20b39a"
        },
        "date": 1791242760867,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2011.684,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 398,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2738.338,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.195,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.263,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "49974392+Nabla7@users.noreply.github.com",
            "name": "Pim Van den Bosch",
            "username": "Nabla7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7554a447c1afbe1df1f825971db0b36ebf518190",
          "message": "Fix RPC shell startup with older IPython versions (#4306)",
          "timestamp": "2026-10-05T23:48:32Z",
          "tree_id": "6cade672a15de7cdb2231e4849e05ebe5c6c2774",
          "url": "https://github.com/dimensionalOS/dimos/commit/7554a447c1afbe1df1f825971db0b36ebf518190"
        },
        "date": 1791244281195,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2023.395,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 399,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2737.96,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.199,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.2,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b8cfd0ff177dc51aaa8926c9c9cbb8ef4b09a69d",
          "message": "ci: require policy from the event base (#4443)",
          "timestamp": "2026-10-06T04:12:32Z",
          "tree_id": "74d0218b27c21508c2b9663d430ed0fadc4cca79",
          "url": "https://github.com/dimensionalOS/dimos/commit/b8cfd0ff177dc51aaa8926c9c9cbb8ef4b09a69d"
        },
        "date": 1791260119979,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2030.07,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 401,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2735.029,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.188,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.164,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0",
          "message": "perf: overlap model loads and heavy imports with the blueprint deploy (#4285)",
          "timestamp": "2026-10-05T22:34:13-07:00",
          "tree_id": "bb16775419f95c779a882f6f37dbf8915f1c0aca",
          "url": "https://github.com/dimensionalOS/dimos/commit/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0"
        },
        "date": 1791265039117,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2018.945,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 401,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2737.967,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.156,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.242,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "bogwi@tutamail.com",
            "name": "Dan Vi",
            "username": "bogwi"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7b1d8ab5dfddafd3a84c39fe58e2ea6c5e873915",
          "message": "Remove skipif_macos_bug from tests that pass on Darwin. (#4204)\n\nCo-authored-by: bogwi <bogdan@dimensional.org>",
          "timestamp": "2026-10-06T23:29:27+08:00",
          "tree_id": "f37afd7e6bb20e5f1d2af6cfeadb0150863dac4c",
          "url": "https://github.com/dimensionalOS/dimos/commit/7b1d8ab5dfddafd3a84c39fe58e2ea6c5e873915"
        },
        "date": 1791301020484,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1785.668,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 398,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2738.009,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.34,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.301,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "lesh@sysphere.org",
            "name": "leshy",
            "username": "leshy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d1f599c5ceb45dedbf89f4722fabb13cf01b4399",
          "message": "chore(codeowners): Andrew/Ivan to codeowners for nav (#4447)",
          "timestamp": "2026-10-06T19:03:44+03:00",
          "tree_id": "4c9b4c35b1217809a7848e48b3b15dd2bd094132",
          "url": "https://github.com/dimensionalOS/dimos/commit/d1f599c5ceb45dedbf89f4722fabb13cf01b4399"
        },
        "date": 1791302797642,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2024.062,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 395,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.666,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.195,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.273,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "07edae3181158fd76f1aac67d56cab2102886427",
          "message": "perf(perception): CLIP embeddings without transformers or torch (#4286)",
          "timestamp": "2026-10-06T20:42:32+03:00",
          "tree_id": "a216723c0b03f90a4356185fd4dd124feb679490",
          "url": "https://github.com/dimensionalOS/dimos/commit/07edae3181158fd76f1aac67d56cab2102886427"
        },
        "date": 1791308740814,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2038.902,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 399,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2732.625,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.348,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.518,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5b37e609adb5736e0eebb9d6bf2b9b1aa43d5571",
          "message": "refactor(agents): SkillResult reports facts (#4404)",
          "timestamp": "2026-10-06T22:41:18+03:00",
          "tree_id": "00c3fc70a15873421253b30b729755f556bf66fd",
          "url": "https://github.com/dimensionalOS/dimos/commit/5b37e609adb5736e0eebb9d6bf2b9b1aa43d5571"
        },
        "date": 1791315847854,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2054.676,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 392,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2737.523,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.301,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.228,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "99a2a4f8a32d2856ee09a2914f6154f2e262d75d",
          "message": "feat(web): add WebRTC video signaling to the protocol and the relay (#4264)",
          "timestamp": "2026-10-06T23:39:01+03:00",
          "tree_id": "cb1e69227c3840c2d2989d0803fbcadec8235e40",
          "url": "https://github.com/dimensionalOS/dimos/commit/99a2a4f8a32d2856ee09a2914f6154f2e262d75d"
        },
        "date": 1791319330696,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1997.613,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 394,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2738.036,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.273,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.189,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4048c568a4e634eeab6f4655f0d94f12db950f57",
          "message": "feat(evals): environments and cases specify which artifacts are given to the evaluated agent (#4384)",
          "timestamp": "2026-10-06T16:39:38-07:00",
          "tree_id": "0adaf0048015b8e5fe3dc16856c8ae534892c80f",
          "url": "https://github.com/dimensionalOS/dimos/commit/4048c568a4e634eeab6f4655f0d94f12db950f57"
        },
        "date": 1791330163310,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2021.508,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 402,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2734.852,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.156,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.214,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4acbaddb43cefe5bd028bdecf97d75d0c6fd7227",
          "message": "perf: stop loading heavy libraries in processes that never use them (#4282)",
          "timestamp": "2026-10-06T17:51:29-07:00",
          "tree_id": "6aa3b934b2ca0f8f58ca0ce46c2cdecfe6cf0c0a",
          "url": "https://github.com/dimensionalOS/dimos/commit/4acbaddb43cefe5bd028bdecf97d75d0c6fd7227"
        },
        "date": 1791334452122,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1601.258,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 394,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2733.152,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.867,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 124.617,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "faf1bed3a5c07b1023f484b7cb544cee5b7908cd",
          "message": "docs: ray tracer (#4411)",
          "timestamp": "2026-10-06T18:24:47-07:00",
          "tree_id": "42dbb668e223683b640b312935e7a23a90774f1e",
          "url": "https://github.com/dimensionalOS/dimos/commit/faf1bed3a5c07b1023f484b7cb544cee5b7908cd"
        },
        "date": 1791336455587,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1616.949,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 402,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2738.458,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.598,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 124.651,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f22803d194135b86435893d2c2395bdbdf76c0d0",
          "message": "fix: resolve native sources for pip installations (#3970)",
          "timestamp": "2026-10-07T09:18:11+03:00",
          "tree_id": "ce94b943392660053c9e74d55833c437ef3bf4ff",
          "url": "https://github.com/dimensionalOS/dimos/commit/f22803d194135b86435893d2c2395bdbdf76c0d0"
        },
        "date": 1791354068479,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1597.902,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 389,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.806,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.895,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 124.455,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "lesh@sysphere.org",
            "name": "leshy",
            "username": "leshy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e7e289830d8e7d79426d4892ecd70f8997ed7922",
          "message": "Ivan/feat/go2 nav fixes (#4449)",
          "timestamp": "2026-10-07T20:54:22+03:00",
          "tree_id": "d72ac943e5cfdf99d4fd672f47e509cbee6857ff",
          "url": "https://github.com/dimensionalOS/dimos/commit/e7e289830d8e7d79426d4892ecd70f8997ed7922"
        },
        "date": 1791395839716,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1515.895,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2727.344,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.375,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.22,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a017963b9372711763e705dfea6acd6542008075",
          "message": "depth2depth: Depth Anything cloud calibrated per pixel to the lidar (#4330)",
          "timestamp": "2026-10-07T20:14:45Z",
          "tree_id": "484e6f084ec4655bd485426b92a884456f15e84f",
          "url": "https://github.com/dimensionalOS/dimos/commit/a017963b9372711763e705dfea6acd6542008075"
        },
        "date": 1791404298704,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1484.125,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 344,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.954,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.363,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.307,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d0e2332679bbc9376304d091863c40c696556933",
          "message": "fix: lazy voxel normal calculations and voxel chunking (#4324)\n\nCo-authored-by: Jeff Hykin <jeff.hykin@gmail.com>",
          "timestamp": "2026-10-07T23:55:20+03:00",
          "tree_id": "ce2d9ee14026daf916e3b314c48b1dae4501b2c0",
          "url": "https://github.com/dimensionalOS/dimos/commit/d0e2332679bbc9376304d091863c40c696556933"
        },
        "date": 1791406706277,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1515.949,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2723.985,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.457,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.187,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "aea28cfce4f7bd02afc4deb1cc9715e5d15fc8b5",
          "message": "feat(imitation): publish validated episode JSON and HUD status (#4393)",
          "timestamp": "2026-10-07T14:10:27-07:00",
          "tree_id": "140c4f8d1c60c8cca3bcbc776e995ffa1f4b9150",
          "url": "https://github.com/dimensionalOS/dimos/commit/aea28cfce4f7bd02afc4deb1cc9715e5d15fc8b5"
        },
        "date": 1791407592139,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1529.824,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 346,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.733,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.277,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.227,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "63036454+ruthwikdasyam@users.noreply.github.com",
            "name": "ruthwikdasyam",
            "username": "ruthwikdasyam"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dbb630f8e298218e9862e03dc24d6ba47080d20c",
          "message": "feat(evals): add six robosuite scenes (#4413)",
          "timestamp": "2026-10-08T01:35:29+03:00",
          "tree_id": "3f26421a2470c9e4a9904862ee1b615589d92df3",
          "url": "https://github.com/dimensionalOS/dimos/commit/dbb630f8e298218e9862e03dc24d6ba47080d20c"
        },
        "date": 1791412703881,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1524.16,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 348,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.093,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.543,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.257,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "63036454+ruthwikdasyam@users.noreply.github.com",
            "name": "ruthwikdasyam",
            "username": "ruthwikdasyam"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "504e582062f1d528366d2e9592cc2a0489d5485d",
          "message": "feat(evals): run evals in Docker containers with dimos evals run --docker (#4225)\n\nCo-authored-by: stash <pomichterstash@gmail.com>",
          "timestamp": "2026-10-07T18:36:14-07:00",
          "tree_id": "be3a192bd2e20368734669dec54a14e189c0e18f",
          "url": "https://github.com/dimensionalOS/dimos/commit/504e582062f1d528366d2e9592cc2a0489d5485d"
        },
        "date": 1791423532440,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1529.922,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2723.867,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.535,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.235,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "63036454+ruthwikdasyam@users.noreply.github.com",
            "name": "ruthwikdasyam",
            "username": "ruthwikdasyam"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "90bf6a1343656069355ddeda64af044d44b733c6",
          "message": "feat(control): publish measured end-effector poses on tf (#4450)",
          "timestamp": "2026-10-08T01:54:57Z",
          "tree_id": "e6078d9e35284860bbf39d87cb8dfdd33f1c0fbc",
          "url": "https://github.com/dimensionalOS/dimos/commit/90bf6a1343656069355ddeda64af044d44b733c6"
        },
        "date": 1791424670168,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1525.473,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 359,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.754,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.273,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.275,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5b0d592517597b01a8c8dcf4883f7caacda83164",
          "message": "r1: enable config  (#4358)",
          "timestamp": "2026-10-08T03:05:47Z",
          "tree_id": "f5b9df8c96047ac0b827975db1a2ce7cafcdcce6",
          "url": "https://github.com/dimensionalOS/dimos/commit/5b0d592517597b01a8c8dcf4883f7caacda83164"
        },
        "date": 1791428930913,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1525.852,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 354,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.714,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.512,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.342,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "63036454+ruthwikdasyam@users.noreply.github.com",
            "name": "ruthwikdasyam",
            "username": "ruthwikdasyam"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "64019369eb536af3cb6690679462d577eac238bc",
          "message": "feat(evals): one raw robot bridge for navigation and manipulation (#4417)",
          "timestamp": "2026-10-08T07:23:56+03:00",
          "tree_id": "9741b6eabfef1d553088b6d117cad29d05b7fc3f",
          "url": "https://github.com/dimensionalOS/dimos/commit/64019369eb536af3cb6690679462d577eac238bc"
        },
        "date": 1791433605155,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1525.828,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 348,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.968,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.59,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.343,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "55f1061cad657fa4d2bf56563007969c6d85f7c4",
          "message": "feat(dataprep): align recordings with explicit feature schemas (#4394)",
          "timestamp": "2026-10-07T21:28:08-07:00",
          "tree_id": "946894f8e7ca74d88b943b4ef88b346ff13f71c5",
          "url": "https://github.com/dimensionalOS/dimos/commit/55f1061cad657fa4d2bf56563007969c6d85f7c4"
        },
        "date": 1791433849816,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1542.641,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2726.512,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.461,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.211,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "pomichterstash@gmail.com",
            "name": "stash",
            "username": "spomichter"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dc80d89b89558e9ef3bcccc7949b20aa3e2b4d3b",
          "message": "feat(go2): teleop-only raw recording blueprint, KeyboardTeleop key presses as Joy (#4478)\n\nCo-authored-by: Krishna_Hundekari <krishna@dimensionalos.com>",
          "timestamp": "2026-10-08T00:15:23-07:00",
          "tree_id": "97f388897a3924fbf5ffdadd3887d99593596bc1",
          "url": "https://github.com/dimensionalOS/dimos/commit/dc80d89b89558e9ef3bcccc7949b20aa3e2b4d3b"
        },
        "date": 1791443909222,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1522.645,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 351,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2723.864,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.566,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.21,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "pomichterstash@gmail.com",
            "name": "stash",
            "username": "spomichter"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "293b92735544a8b8d14972b8271bdd92668f1e87",
          "message": "cloud: dimos data upload sends a spatial preview for the console (#4458)",
          "timestamp": "2026-10-08T02:39:27-07:00",
          "tree_id": "b9e92bcfaa40fe5dad1f9aa60e26779409d612e6",
          "url": "https://github.com/dimensionalOS/dimos/commit/293b92735544a8b8d14972b8271bdd92668f1e87"
        },
        "date": 1791452543003,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1517.016,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 351,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2723.871,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.312,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.24,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "feb30ea34a6392c985d384d1b6d510029376f6a9",
          "message": "chore: clean debug logging (#4460)\n\nCo-authored-by: leshy <lesh@sysphere.org>",
          "timestamp": "2026-10-08T10:21:45Z",
          "tree_id": "6bfcd75cc70d7224e9a5cc9eedcfc74841e8dada",
          "url": "https://github.com/dimensionalOS/dimos/commit/feb30ea34a6392c985d384d1b6d510029376f6a9"
        },
        "date": 1791455089422,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1508.242,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2723.875,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.312,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.235,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "pomichterstash@gmail.com",
            "name": "stash",
            "username": "spomichter"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4828b504f8fc0aa64968b79f75608f7ca67f092e",
          "message": "fix(cloud): preview picks the right streams; a failed timelapse keeps the preview (#4480)",
          "timestamp": "2026-10-08T03:43:42-07:00",
          "tree_id": "7a72ea21bb06eb04b73455cbc652e0eb4f8a369a",
          "url": "https://github.com/dimensionalOS/dimos/commit/4828b504f8fc0aa64968b79f75608f7ca67f092e"
        },
        "date": 1791456393405,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1517.84,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 350,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.728,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.316,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.281,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dad5ad7cb4277ee4b266f415067bda6e352ab419",
          "message": "feat: go2 and mid360 sim (#4441)",
          "timestamp": "2026-10-08T17:29:36Z",
          "tree_id": "7ce800f6d86d46fbf13acf1a4da40de4ae5091be",
          "url": "https://github.com/dimensionalOS/dimos/commit/dad5ad7cb4277ee4b266f415067bda6e352ab419"
        },
        "date": 1791480753033,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1514.387,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 349,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.717,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.332,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.274,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "0761d4ee215821f4116ff53e88465df4149507a4",
          "message": "Updated code ownership for CC to all code (#4444)",
          "timestamp": "2026-10-08T15:13:55-07:00",
          "tree_id": "9c92f7ddc229b60dc86ac6c5b9c53abdbf1ee7c2",
          "url": "https://github.com/dimensionalOS/dimos/commit/0761d4ee215821f4116ff53e88465df4149507a4"
        },
        "date": 1791497797461,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1520.477,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 348,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2727.366,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.324,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.28,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6ed445f0f45898f8466c0c5ae2850ceb4b0d5ad3",
          "message": "feat(control): add control contract package (#4263)",
          "timestamp": "2026-10-08T15:14:44-07:00",
          "tree_id": "b0d7271c8eeeeea8c7496293f8049cedd6668315",
          "url": "https://github.com/dimensionalOS/dimos/commit/6ed445f0f45898f8466c0c5ae2850ceb4b0d5ad3"
        },
        "date": 1791497846087,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1524.723,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 358,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.216,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.312,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 112.257,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "9197ea5b1dca07c6c10f4b1141fd627754c146c7",
          "message": "perf(core): warm up stream types and skills right after a module is constructed (#4284)",
          "timestamp": "2026-10-09T01:22:46+03:00",
          "tree_id": "d65e539e9f3553629d5d8c833e42ae2372e184bb",
          "url": "https://github.com/dimensionalOS/dimos/commit/9197ea5b1dca07c6c10f4b1141fd627754c146c7"
        },
        "date": 1791498322828,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1576.41,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 345,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.658,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.465,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.776,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c53724a060988f279c347e954beb0e8916a3ecdb",
          "message": "fix: remove blank g1 image (#4486)",
          "timestamp": "2026-10-09T01:09:09Z",
          "tree_id": "e4e9ecbfb80ced608e9322163d045b3696b02f3e",
          "url": "https://github.com/dimensionalOS/dimos/commit/c53724a060988f279c347e954beb0e8916a3ecdb"
        },
        "date": 1791508319015,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1574.797,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 349,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.111,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.742,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.517,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c25d33135d422464b0d6c09135c8b51176f4ed61",
          "message": "feat(web): receive WebRTC video tracks in the SDK and the cockpit (#4265)",
          "timestamp": "2026-10-09T07:18:20+03:00",
          "tree_id": "8efc4e5b2c1e439105b381d4bdaa66d18e14cb57",
          "url": "https://github.com/dimensionalOS/dimos/commit/c25d33135d422464b0d6c09135c8b51176f4ed61"
        },
        "date": 1791519656988,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1577.66,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 349,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.102,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.652,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.604,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "pomichterstash@gmail.com",
            "name": "stash",
            "username": "spomichter"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4d7496165a5bb3771967bcbf1c5ee82a70611a0c",
          "message": "cloud: console preview carries joystick input (#4491)",
          "timestamp": "2026-10-08T21:33:44-07:00",
          "tree_id": "39818ff87d92c4a3d5844a423f91c399af065b41",
          "url": "https://github.com/dimensionalOS/dimos/commit/4d7496165a5bb3771967bcbf1c5ee82a70611a0c"
        },
        "date": 1791520605630,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1557.562,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 354,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.094,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.441,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.724,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "bogwi@tutamail.com",
            "name": "Dan Vi",
            "username": "bogwi"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "0147381f1a28a3dd9df8bce93de409baf1f85ca5",
          "message": "fix relay e2e timeout mac test (#4488)",
          "timestamp": "2026-10-09T17:02:02+03:00",
          "tree_id": "c88f9043857d34fba2698e267b16ca5c18267730",
          "url": "https://github.com/dimensionalOS/dimos/commit/0147381f1a28a3dd9df8bce93de409baf1f85ca5"
        },
        "date": 1791554689446,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1577.371,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 342,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2721.58,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.664,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.554,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e6e7a6d684294434d7523cfc7309a2169e3621c5",
          "message": "feat: go2 nav sim (#4481)",
          "timestamp": "2026-10-09T19:29:58Z",
          "tree_id": "228235b9f59609f56339e3be563e5202e9c7e905",
          "url": "https://github.com/dimensionalOS/dimos/commit/e6e7a6d684294434d7523cfc7309a2169e3621c5"
        },
        "date": 1791574368850,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1574.465,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.773,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.723,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.776,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4493f5ca86faf78f7e7ad681ee9bba3c88345892",
          "message": "Add dimsim ready state to avoid race condition (#4495)",
          "timestamp": "2026-10-09T19:59:55Z",
          "tree_id": "a760abada8b4ed224df551927bc98ec0b2a241dc",
          "url": "https://github.com/dimensionalOS/dimos/commit/4493f5ca86faf78f7e7ad681ee9bba3c88345892"
        },
        "date": 1791576167028,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1567.539,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 350,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.715,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.449,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "647a9b1ceb42df774333588e4d2a6e90206ca652",
          "message": "fix: test_walk_forward (#4501)",
          "timestamp": "2026-10-09T18:10:08-07:00",
          "tree_id": "60368b1ec1cc704349c60471cbf9336c6e65305a",
          "url": "https://github.com/dimensionalOS/dimos/commit/647a9b1ceb42df774333588e4d2a6e90206ca652"
        },
        "date": 1791594767216,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1580.527,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 347,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.001,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.719,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.548,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "bogwi@tutamail.com",
            "name": "Dan Vi",
            "username": "bogwi"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "622d924eceb7c5b4dcfed12d000c42f5268e4cc2",
          "message": "refactor(dannav): rework DanHolonomicTC tracking (#4493)",
          "timestamp": "2026-10-10T11:10:05+09:00",
          "tree_id": "d5912c1fc6fcd7f19e6995b3215f2f830746eabd",
          "url": "https://github.com/dimensionalOS/dimos/commit/622d924eceb7c5b4dcfed12d000c42f5268e4cc2"
        },
        "date": 1791598362182,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1575.375,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 356,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.999,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.457,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.904,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "649b045296ef5d059dfcdfe8adfb13dd72e9d8aa",
          "message": "Move a750-control out of our extras (#4494)",
          "timestamp": "2026-10-09T20:59:32-07:00",
          "tree_id": "08403c2f50ad0d19e0440025e03043244c269f32",
          "url": "https://github.com/dimensionalOS/dimos/commit/649b045296ef5d059dfcdfe8adfb13dd72e9d8aa"
        },
        "date": 1791604938381,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1570.781,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 345,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.499,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.676,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.669,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8b17f2ff38979a79112ad8c38c9a5e88bcde4c77",
          "message": "feat(control): add description presets and limits_from_urdf (#4273)",
          "timestamp": "2026-10-09T21:27:27-07:00",
          "tree_id": "1c372b362014292b672cfa0b7c6f32d661bd90f4",
          "url": "https://github.com/dimensionalOS/dimos/commit/8b17f2ff38979a79112ad8c38c9a5e88bcde4c77"
        },
        "date": 1791606610727,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1582.812,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 360,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2721.697,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.617,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.471,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "066d3f3fc21a0785b7fe1c5f40e572c60cd2e4d6",
          "message": "Migrate planning and point-cloud filtering to RoboPlan 0.7 (#4420)",
          "timestamp": "2026-10-10T11:47:07-07:00",
          "tree_id": "8acdf47d07bda48b8ae978cabe6ad2a6e52f2697",
          "url": "https://github.com/dimensionalOS/dimos/commit/066d3f3fc21a0785b7fe1c5f40e572c60cd2e4d6"
        },
        "date": 1791658192819,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1572.152,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 348,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.138,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.676,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.648,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1e7b4918bc626de4fa6592fb023d4b3aee932754",
          "message": "test: give the wrong-layout policy blob cases readable ids (#4516)",
          "timestamp": "2026-10-11T02:37:32Z",
          "tree_id": "02d1a6455c8197bc0dbec630dcce472fafd7c082",
          "url": "https://github.com/dimensionalOS/dimos/commit/1e7b4918bc626de4fa6592fb023d4b3aee932754"
        },
        "date": 1791686414489,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1569.52,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.016,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.492,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.634,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fd989ff70f0c86a1e5c53f9414944c9fc61dd963",
          "message": "Update ROS image (#4496)",
          "timestamp": "2026-10-12T17:46:28+01:00",
          "tree_id": "e92a9bffc74a22e9b20b69f5a4b3abd4429f7fe0",
          "url": "https://github.com/dimensionalOS/dimos/commit/fd989ff70f0c86a1e5c53f9414944c9fc61dd963"
        },
        "date": 1791823758912,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1578.188,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 349,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.046,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.664,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.658,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "475fad598448ab125fb8ce19fb8496b759b7d58b",
          "message": "feat(web): send video as WebRTC tracks from the relay bridge (#4267)",
          "timestamp": "2026-10-12T19:57:29+03:00",
          "tree_id": "74fc49bc2ff5427b472f56aef685e333cfd5df7b",
          "url": "https://github.com/dimensionalOS/dimos/commit/475fad598448ab125fb8ce19fb8496b759b7d58b"
        },
        "date": 1791824425045,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1597.328,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 339,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2725.136,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.492,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.562,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c0d14726f2f2b88ba4d901d4fd3fc83261a1fd8b",
          "message": "refactor: deprecate nav eval tool (#4438)",
          "timestamp": "2026-10-12T19:25:40Z",
          "tree_id": "2079233757d5c6a1146bc57d3eca59d7cc4325cc",
          "url": "https://github.com/dimensionalOS/dimos/commit/c0d14726f2f2b88ba4d901d4fd3fc83261a1fd8b"
        },
        "date": 1791833305704,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1573.219,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 346,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.012,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.676,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.427,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "908fc40a07056d58453b6a219d0c82e9b684dead",
          "message": "feat(evals): find all cones in building (#4414)",
          "timestamp": "2026-10-12T15:27:03-07:00",
          "tree_id": "2456f169c1300a4c29b370d28a2546ab843d59ab",
          "url": "https://github.com/dimensionalOS/dimos/commit/908fc40a07056d58453b6a219d0c82e9b684dead"
        },
        "date": 1791844192682,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1569.934,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 347,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.663,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.484,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.415,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "288875102ff85a0994e2c6f4b5d63d7f0734dfd0",
          "message": "Move why above what in PR descriptions (#4514)",
          "timestamp": "2026-10-13T06:19:18+03:00",
          "tree_id": "2f7599093fe8aedf4b63f7ea1f11d8364c98383d",
          "url": "https://github.com/dimensionalOS/dimos/commit/288875102ff85a0994e2c6f4b5d63d7f0734dfd0"
        },
        "date": 1791861739978,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1549.234,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 348,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.636,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.504,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.642,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "26ccb5d88e8e1bd674e8339e821a8a8bc5880b48",
          "message": "depth2depth: optimize cloud calculations (#4549)",
          "timestamp": "2026-10-12T21:27:18-07:00",
          "tree_id": "fc6b5566a7b6dcf76269d6576589bc6a37959db4",
          "url": "https://github.com/dimensionalOS/dimos/commit/26ccb5d88e8e1bd674e8339e821a8a8bc5880b48"
        },
        "date": 1791865817493,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1559.621,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 347,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.735,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.449,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.618,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3b6d5eafe66bcf8378b50b86c422a48a05916321",
          "message": "chore: remove shm transports (#4550)",
          "timestamp": "2026-10-13T08:11:17+03:00",
          "tree_id": "7658beeeb2eeda79d3bf59c16fc9dd910d88041e",
          "url": "https://github.com/dimensionalOS/dimos/commit/3b6d5eafe66bcf8378b50b86c422a48a05916321"
        },
        "date": 1791868463828,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1558.273,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 346,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.048,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.465,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.506,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "wuy691128@gmail.com",
            "name": "poorwym",
            "username": "poorwym"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "bd82b3dd15c2c7b0ebf59bfb158aa4626b6570ac",
          "message": "Yimo/feat/host fragment (#3967)\n\nCo-authored-by: yimo <yimo@dimensional.org>\nCo-authored-by: leshy <lesh@sysphere.org>",
          "timestamp": "2026-10-13T12:17:18Z",
          "tree_id": "c6ae6671b266eb495aa60a20b8893428ecd483d8",
          "url": "https://github.com/dimensionalOS/dimos/commit/bd82b3dd15c2c7b0ebf59bfb158aa4626b6570ac"
        },
        "date": 1791894009351,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1562.855,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 356,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.479,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.43,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.529,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "bogwi@tutamail.com",
            "name": "Dan Vi",
            "username": "bogwi"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "19e9c0a198af61be1a1901b4027c00af2a2acdf2",
          "message": "feat(models): add SigLIP2 embedding wrapper (#4539)\n\nCo-authored-by: bogwi <bogdan@dimensional.org>",
          "timestamp": "2026-10-13T19:13:45+03:00",
          "tree_id": "9d121bcb3a2cc2096fc34e68a313dd293d22c6ea",
          "url": "https://github.com/dimensionalOS/dimos/commit/19e9c0a198af61be1a1901b4027c00af2a2acdf2"
        },
        "date": 1791908195456,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1569.781,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1119/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 346,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1119/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.693,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1119/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.805,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1119/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.585,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1119/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "lesh@sysphere.org",
            "name": "leshy",
            "username": "leshy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a84700b089b9948831bb489d643a29f5d3d47cda",
          "message": "feat(mapping): live mesh for go2 (#4497)",
          "timestamp": "2026-10-13T17:13:22Z",
          "tree_id": "d1f9a3b9da42363b0568a3fc455ea27772421cc6",
          "url": "https://github.com/dimensionalOS/dimos/commit/a84700b089b9948831bb489d643a29f5d3d47cda"
        },
        "date": 1791911773333,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1565.832,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 354,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2723.766,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.664,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.717,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e132bc5290574eb22355afe5b1ca580c5a2036f6",
          "message": "refactor(habitat): extract asset paths to constants (#4459)",
          "timestamp": "2026-10-13T17:38:01Z",
          "tree_id": "504fec46e75ddc8df0785c56ab8d2e81a0e54184",
          "url": "https://github.com/dimensionalOS/dimos/commit/e132bc5290574eb22355afe5b1ca580c5a2036f6"
        },
        "date": 1791913264590,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1553.996,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 347,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.066,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.477,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.448,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1c5e5addce8619e42f1536f2385a700262547d8a",
          "message": "feat(sim): the 555 office as the default MuJoCo",
          "timestamp": "2026-10-13T13:05:36-07:00",
          "tree_id": "7efb6a3b371f66d7be1ed9ecfa2ef09e7127559f",
          "url": "https://github.com/dimensionalOS/dimos/commit/1c5e5addce8619e42f1536f2385a700262547d8a"
        },
        "date": 1791922112290,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1566.473,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 355,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2721.676,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.473,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.395,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "lesh@sysphere.org",
            "name": "leshy",
            "username": "leshy"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "49fa9f8c33e4994ebac8ee266303732e1d7c7983",
          "message": "docs: remove hosted architecture draft (#4556)",
          "timestamp": "2026-10-13T20:16:06Z",
          "tree_id": "e23068f5f7610b1a12eac5bd69c4681bc3e94e13",
          "url": "https://github.com/dimensionalOS/dimos/commit/49fa9f8c33e4994ebac8ee266303732e1d7c7983"
        },
        "date": 1791922734378,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1567.039,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 340,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.98,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.477,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.603,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "263b0591c9980d9e11c605a86811216bbdb8018f",
          "message": "feat(porcelain): add remote stream discovery and subscription  (#4492)\n\nCo-authored-by: Paul Nechifor <paul@nechifor.net>",
          "timestamp": "2026-10-14T00:19:35+03:00",
          "tree_id": "925b77c81287cfe93fa65ae8a8e71b7988f1fb63",
          "url": "https://github.com/dimensionalOS/dimos/commit/263b0591c9980d9e11c605a86811216bbdb8018f"
        },
        "date": 1791926540790,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1588.074,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 364,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.045,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.484,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.456,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5d97134cc1fda66e9f26127f59b3b78687648e31",
          "message": "r1pro: boot the vendor stack with one setup script (#4558)",
          "timestamp": "2026-10-13T20:12:22-07:00",
          "tree_id": "49282bc854c2cd512c2524f8a16a7909e524d32b",
          "url": "https://github.com/dimensionalOS/dimos/commit/5d97134cc1fda66e9f26127f59b3b78687648e31"
        },
        "date": 1791947718592,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1570.738,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 349,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.098,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.664,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.364,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "alxbouchard@gmail.com",
            "name": "alxbouchard",
            "username": "alxbouchard"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "0f65afc34c6012e2d7924e620fb24fcb9f171510",
          "message": "fix(memory): record depth images losslessly instead of as JPEG (#4554)\n\nCo-authored-by: leshy <lesh@sysphere.org>",
          "timestamp": "2026-10-14T03:31:34Z",
          "tree_id": "e92c0f31d14486b530b83d79cfc64d96cb53d93c",
          "url": "https://github.com/dimensionalOS/dimos/commit/0f65afc34c6012e2d7924e620fb24fcb9f171510"
        },
        "date": 1791948867752,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1557.73,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 368,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.332,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.461,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.606,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "moshi@dimensional.org",
            "name": "moshiii-wei",
            "username": "moshiii-wei"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "886af5be9b612e6f479075da01b5c500fd25c7dd",
          "message": "feat(evals): manifest-driven LIBERO-PRO/RoboCasa MuJoCo smoke (#4517)\n\nCo-authored-by: Moshi Wei <wmswms938@gmail.com>",
          "timestamp": "2026-10-13T23:14:01-07:00",
          "tree_id": "69cdcfe362ef8a336dd7d063c83fdb227b370b0b",
          "url": "https://github.com/dimensionalOS/dimos/commit/886af5be9b612e6f479075da01b5c500fd25c7dd"
        },
        "date": 1791958609369,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1586.348,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 851/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 365,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 851/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2718.603,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 851/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.488,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 851/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.035,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 851/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "bogwi@tutamail.com",
            "name": "Dan Vi",
            "username": "bogwi"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5a694d6dcbcbce868b14eccbe5be60c4c623c4c9",
          "message": "refactor perception stack to modular design (#4552)",
          "timestamp": "2026-10-14T16:02:28+09:00",
          "tree_id": "0b75da272bbecf11f3551ae344972eddb6dca9cc",
          "url": "https://github.com/dimensionalOS/dimos/commit/5a694d6dcbcbce868b14eccbe5be60c4c623c4c9"
        },
        "date": 1791961500995,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1570.531,
            "unit": "MB",
            "extra": "cpu: Neoverse-V3; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 351,
            "unit": "threads",
            "extra": "cpu: Neoverse-V3; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.212,
            "unit": "MB",
            "extra": "cpu: Neoverse-V3; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.477,
            "unit": "MB",
            "extra": "cpu: Neoverse-V3; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.491,
            "unit": "G",
            "extra": "cpu: Neoverse-V3; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3faae1f93ed44e1119ec8bd5916132bed34b2219",
          "message": "ci: reject relative Python imports with Ruff (#4563)",
          "timestamp": "2026-10-14T15:44:59+03:00",
          "tree_id": "903c6d32f72d57a678d4709fab16cb3c1d19aff4",
          "url": "https://github.com/dimensionalOS/dimos/commit/3faae1f93ed44e1119ec8bd5916132bed34b2219"
        },
        "date": 1791982081148,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1589.676,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 360,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.636,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.734,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.588,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5a7149c401eabbba8e2e676bff54ef5859b3b69c",
          "message": "dimOS Desktop contract: dimos.yaml and install.sh fixes (#4428)",
          "timestamp": "2026-10-15T01:44:37+03:00",
          "tree_id": "607d756b144bd061e3315456326a7c6f526013da",
          "url": "https://github.com/dimensionalOS/dimos/commit/5a7149c401eabbba8e2e676bff54ef5859b3b69c"
        },
        "date": 1792018051171,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1564.09,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 360,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.107,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.484,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.57,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "19c7db2e9056dd6454e82fcfad341f343b5aa8f7",
          "message": "fix(lcm): allow shutdown from dispatch callbacks (#4515)",
          "timestamp": "2026-10-15T01:27:34Z",
          "tree_id": "7874f9e01368ea7e88648928373e82679c23d26f",
          "url": "https://github.com/dimensionalOS/dimos/commit/19c7db2e9056dd6454e82fcfad341f343b5aa8f7"
        },
        "date": 1792027823695,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1559.703,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1120/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 355,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1120/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.727,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1120/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.457,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1120/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.463,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1120/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "paul@nechifor.net",
            "name": "Paul Nechifor",
            "username": "paul-nechifor"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f21694937364cf756f7bc2917bbf2191cd04d86b",
          "message": "test: delete 37 tests that never run in CI (#4569)",
          "timestamp": "2026-10-15T04:35:20+03:00",
          "tree_id": "59e9900773b166679dba141ed4907cab738ff276",
          "url": "https://github.com/dimensionalOS/dimos/commit/f21694937364cf756f7bc2917bbf2191cd04d86b"
        },
        "date": 1792028288478,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1572.246,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 352,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.16,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.773,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.59,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5478eb724312b7793a3837d06dc5e4ebc35f6a98",
          "message": "r1pro: Point-LIO odometry on the chassis Mid-360, as r1pro-pointlio (#4360)",
          "timestamp": "2026-10-15T02:14:31Z",
          "tree_id": "e55b47e1382bea3e7c0090eb93c4c455b9f82cea",
          "url": "https://github.com/dimensionalOS/dimos/commit/5478eb724312b7793a3837d06dc5e4ebc35f6a98"
        },
        "date": 1792030636560,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1598.121,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 362,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2727.436,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.438,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.434,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "624bba27685fe0cef3e4a0a1cc08cad499f4899e",
          "message": "Merge PR #1: G1 SONIC arm skills, imported gestures, voice lines\n\nG1 SONIC arm skills (Tier 1, forearm pointing, 0.75 rad/s), imported and hand-authored gestures with Dex3 hands, gesture voice lines, and the CI fixes that make the suite pass on loaded runners.",
          "timestamp": "2026-10-26T18:59:30+09:00",
          "tree_id": "540d80dc530a13051eb7f2fa48124cfc583db70a",
          "url": "https://github.com/caisarl76/dimos/commit/624bba27685fe0cef3e4a0a1cc08cad499f4899e"
        },
        "date": 1793008946346,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1605.117,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 360,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.032,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.695,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.116,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76",
            "name": "Jihun Kim"
          },
          "committer": {
            "email": "caisarl76",
            "name": "Jihun Kim"
          },
          "distinct": true,
          "id": "9908f3af9ecff5835761cee30e15dacaed588f4f",
          "message": "ci: split the Linux suite in two, bound each test, gate on the benchmark\n\n- ci-complete now needs benchmark itself: a failed benchmark skipped\n  benchmark-track, and the fork allowed that skip, so the gate could\n  pass. benchmark may skip only where the path filter skips it.\n- The hosted suite runs with --timeout=120 (addopts' 600 s outlasted\n  the job's 20 min), so a hung test fails with its stacks dumped\n  instead of the job being cancelled. Tests with a timeout mark keep it.\n- Each tests leg runs one of two partitions: dimos/robot, dimos/core\n  and dimos/protocol (about half the measured time), or the rest of\n  dimos/ with those ignored, so the parts always cover the whole suite.\n  Coverage is measured on Python 3.12 only, per partition.",
          "timestamp": "2026-10-26T19:18:31+09:00",
          "tree_id": "6c403fb9729506c4f7cb2150d5eaec8983349915",
          "url": "https://github.com/caisarl76/dimos/commit/9908f3af9ecff5835761cee30e15dacaed588f4f"
        },
        "date": 1793010083241,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1583.766,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 366,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.07,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.738,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.152,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "26010a9bb34eebad948ff304ef8e1ae9474dbd6c",
          "message": "Merge PR #3: correct the coverage comment; run the Livox e2e once, on Linux x86_64\n\nci: correct the coverage comment; run the Livox e2e once, on Linux x86_64",
          "timestamp": "2026-10-26T19:47:33+09:00",
          "tree_id": "093c4d35243fce5224ea26f5e133958dd39048e3",
          "url": "https://github.com/caisarl76/dimos/commit/26010a9bb34eebad948ff304ef8e1ae9474dbd6c"
        },
        "date": 1793011873241,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1567.648,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 370,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.081,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.453,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.906,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b69fe01f2f7396523a5edcf8138749947572535f",
          "message": "Merge PR #2: drive two G1s from one DimOS process with synchronized arm skills",
          "timestamp": "2026-10-26T20:24:07+09:00",
          "tree_id": "58451d3672398f35c673897092b4abb19a3e18d8",
          "url": "https://github.com/caisarl76/dimos/commit/b69fe01f2f7396523a5edcf8138749947572535f"
        },
        "date": 1793014248343,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1607.629,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 359,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.485,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.676,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.255,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f011a42f6d718ae1783743a3426a0ef0f292fa01",
          "message": "Merge PR #5: collision check tolerates a pair resting just outside the 2 cm reach",
          "timestamp": "2026-10-27T10:39:49+09:00",
          "tree_id": "a1dc2c6e604bf8575acaa139ba0a72baf642f89e",
          "url": "https://github.com/caisarl76/dimos/commit/f011a42f6d718ae1783743a3426a0ef0f292fa01"
        },
        "date": 1793065363229,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1585.539,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 364,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.68,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.512,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.942,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4c3bdf6eb5682878144881cd175b5ce593fa5736",
          "message": "Merge PR #8: rerun the dual session tests on a lease stall",
          "timestamp": "2026-10-27T11:26:59+09:00",
          "tree_id": "ec0b0b91ce522ec9c34dde7d752e793c2b66a0d7",
          "url": "https://github.com/caisarl76/dimos/commit/4c3bdf6eb5682878144881cd175b5ce593fa5736"
        },
        "date": 1793068187768,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1572.68,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 351,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.722,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.484,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.941,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "09d52fd1da6faebc5def00c6c548e02e478af977",
          "message": "Merge PR #7: waist motion for invite and thanks, and a new bow gesture",
          "timestamp": "2026-10-27T11:42:10+09:00",
          "tree_id": "d62ee85badd7b184a6549510622e0c6d029d5d0f",
          "url": "https://github.com/caisarl76/dimos/commit/09d52fd1da6faebc5def00c6c548e02e478af977"
        },
        "date": 1793069105250,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1579.453,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 354,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.132,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.5,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.922,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dbf17ec64eea8d775a4483df0470674b127f844d",
          "message": "Merge PR #6: 223 greeter plays demo gestures and lines in random order",
          "timestamp": "2026-10-27T14:15:48+09:00",
          "tree_id": "0941a4956d894b8bbfeab7e615dcdd09a0e59579",
          "url": "https://github.com/caisarl76/dimos/commit/dbf17ec64eea8d775a4483df0470674b127f844d"
        },
        "date": 1793078321766,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1586.82,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 359,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2725.32,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.504,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.128,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1121/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6c96314bce566ed0ac9d7616a7e5f67a64171285",
          "message": "Merge PR #9: per-robot leg margin for the wide-leg gestures (222 at 0.25); report the closest leg\n\nfeat(g1): per-robot leg margin for the wide-leg gestures (222 at 0.25); report the closest leg",
          "timestamp": "2026-10-27T14:26:43+09:00",
          "tree_id": "3b05bea03392cc27147596f7fabdffbf0214bfd2",
          "url": "https://github.com/caisarl76/dimos/commit/6c96314bce566ed0ac9d7616a7e5f67a64171285"
        },
        "date": 1793078974000,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1574.234,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 368,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.668,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.633,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.122,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "014a025d2dd6485ca3ac06c3accbbf8ab229b85a",
          "message": "Merge PR #10: thanks holds the hands in front of the belly",
          "timestamp": "2026-10-27T14:58:54+09:00",
          "tree_id": "5bf132344e6ab78bb4c22605e8d68b4a699ce938",
          "url": "https://github.com/caisarl76/dimos/commit/014a025d2dd6485ca3ac06c3accbbf8ab229b85a"
        },
        "date": 1793080915643,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1554.426,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 359,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2721.601,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.504,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.162,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "23958ae3ded2cf05f29a6367984c94cf4606ace1",
          "message": "Merge PR #11: let entrance_right's stand-in run dip to 6 floor contacts\n\ntest(g1): let entrance_right's stand-in run dip to 6 floor contacts",
          "timestamp": "2026-10-27T15:20:47+09:00",
          "tree_id": "4d71d203c92c80eaa39b0450ef954ad38ea684a0",
          "url": "https://github.com/caisarl76/dimos/commit/23958ae3ded2cf05f29a6367984c94cf4606ace1"
        },
        "date": 1793082228721,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1581.867,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 365,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2725.274,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.496,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.956,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4f72f78fefbfc8f23215a0b1e5644702b5883a89",
          "message": "Merge PR #12: offer 223's two new voice lines in the dual-arm prompt",
          "timestamp": "2026-10-27T15:28:20+09:00",
          "tree_id": "545e9aa1f88cfd47eb3b143d0f225b23ef6b2ae8",
          "url": "https://github.com/caisarl76/dimos/commit/4f72f78fefbfc8f23215a0b1e5644702b5883a89"
        },
        "date": 1793082683419,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1574.43,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 355,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.73,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.484,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.937,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "2adb8ddc0acab333074ec7b2af656188b06b9c09",
          "message": "Merge PR #4: Linux 3.12 as the fork's PR target, a nightly full matrix, apt cache\n\nci: Linux 3.12 as the fork's PR target, a nightly full matrix, apt cache",
          "timestamp": "2026-10-27T15:52:22+09:00",
          "tree_id": "18d6b22ea871dde55b9181ea51d2164ff633b803",
          "url": "https://github.com/caisarl76/dimos/commit/2adb8ddc0acab333074ec7b2af656188b06b9c09"
        },
        "date": 1793084115241,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1585.145,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 359,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.044,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.477,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.1,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ab714fe053194537357e7757d0aca92b1e1d06cb",
          "message": "Merge PR #13: docker compose for 223's greeter stack\n\nbuild(g1): docker compose for 223's greeter stack",
          "timestamp": "2026-10-27T18:45:40+09:00",
          "tree_id": "07666d8fa952d460887d5923210479a392dd7571",
          "url": "https://github.com/caisarl76/dimos/commit/ab714fe053194537357e7757d0aca92b1e1d06cb"
        },
        "date": 1793094511308,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1602.445,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 361,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.703,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.477,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.111,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c1c0277ebd069654a8dc4e60724f43bd2cc734c9",
          "message": "Merge PR #14: one compose project for the demo's workstation side\n\nbuild(g1): one compose project for the demo's workstation side",
          "timestamp": "2026-10-27T19:19:22+09:00",
          "tree_id": "ef86b04fabd07d1b78ce2bcf48488405be8380b4",
          "url": "https://github.com/caisarl76/dimos/commit/c1c0277ebd069654a8dc4e60724f43bd2cc734c9"
        },
        "date": 1793096528129,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1586.496,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 364,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.033,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.602,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.92,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7e04b864462bb87f513dafe0e488f895cf953d4c",
          "message": "Merge PR #15: point the G1 index at the docker/g1-demo compose project\n\ndocs(g1): point the G1 index at the docker/g1-demo compose project",
          "timestamp": "2026-10-28T09:31:24+09:00",
          "tree_id": "b984bf91edd18d4e811b1ac4d0d1b9adcbdd0681",
          "url": "https://github.com/caisarl76/dimos/commit/7e04b864462bb87f513dafe0e488f895cf953d4c"
        },
        "date": 1793147658562,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1574.996,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 363,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.058,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.469,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.152,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ecdd6bfa6b66a929003f6417f8f4ae661fdc8d56",
          "message": "Merge PR #16: demo.sh opens the demo stack and a tmux session of runbook terminals\n\nbuild(g1): demo.sh opens the demo stack and a tmux session of runbook terminals",
          "timestamp": "2026-10-28T10:26:01+09:00",
          "tree_id": "24cdaa32d069c68bdc853d05516f0e1440c3d804",
          "url": "https://github.com/caisarl76/dimos/commit/ecdd6bfa6b66a929003f6417f8f4ae661fdc8d56"
        },
        "date": 1793150933169,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1612.875,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 363,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.068,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.441,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.815,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1118/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "82052e475a79f0532764d8e611539e2b4850862d",
          "message": "Merge pull request #17 from caisarl76/jihun/build/g1-demo-panes\n\nbuild(g1): demo.sh groups the runbook terminals into one window per robot",
          "timestamp": "2026-10-28T10:33:14+09:00",
          "tree_id": "99bbc5500d2a9711e58c7d001726b6212c9f7965",
          "url": "https://github.com/caisarl76/dimos/commit/82052e475a79f0532764d8e611539e2b4850862d"
        },
        "date": 1793151368988,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1578.688,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 362,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2722.643,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.492,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 114.906,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1117/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "caisarl76@gmail.com",
            "name": "Jihun Kim(김지훈)",
            "username": "caisarl76"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c11692950cc8638248f3311d1c7533c34f7965ad",
          "message": "Merge PR #19: demo.sh runs jetson_clocks before 223's controller",
          "timestamp": "2026-10-28T11:19:05+09:00",
          "tree_id": "553d82a104fc54495ef12ed94b668ddac7654970",
          "url": "https://github.com/caisarl76/dimos/commit/c11692950cc8638248f3311d1c7533c34f7965ad"
        },
        "date": 1793154169342,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 1564.625,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 367,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2724.746,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 1.453,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 115.121,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 853/855; perf counted 100.0%"
          }
        ]
      }
    ]
  }
}