
git rev-parse HEAD
git submodule update --init --recursive

Remove-Item -Recurse -Force cmake_build
New-Item -ItemType Directory -Path cmake_build

Write-Host "cmake build"
Set-Location cmake_build
$str = 'cmake -G "Visual Studio 15 2017"'
$cmake_cmd = $str + " -A x64 -DCMAKE_BUILD_TYPE=RELEASE"
Write-Host "cmake_cmd = $cmake_cmd"
Invoke-Expression  $cmake_cmd

Write-Host "msbuild build OrbbecSDK"
cmake --build . --target ALL_BUILD  --config Release

Write-Host "msbuild build INSTALL"
cmake --build . --target INSTALL  --config Release

# Write-Host "install win10 package"
# $package_name = Get-Content "../package_name.txt"
# Write-Host "package name=$package_name"

# $sname = ($package_name -split "_")[3]
# $sw_version = ($package_name -split "_")[-1]
# Write-Host "sname=$sname, sw_version=$sw_version"

# if ($releaseType -eq "Debug") {
#     $package_name = "$package_name_$releaseType"
# }
# Write-Host "cpack package: $package_name"
# cpack
# $nsis_package_name = "$package_name.exe"
# Write-Host "NSIS package: $nsis_package_name"
# $zip_package_name = "$package_name.zip"
# Write-Host "zip package: $zip_package_name"

# Write-Host "publish package ..."
# Remove-Item -Recurse -Force install
# Write-Host "$PWD\install\publish\$sw_version\C&C++\$sname\"
# New-Item -ItemType Directory -Path "$PWD\install\publish\$sw_version\C&C++\$sname\"
# copy .\$nsis_package_name "$PWD\install\publish\$sw_version\C&C++\$sname\"
# copy .\$zip_package_name "$PWD\install\publish\$sw_version\C&C++\$sname\"
# scp -r ./install/publish/*  root@10.20.20.19:/var/lib/jenkins/workspace/publish/SDK/OrbbecSDK/
# Write-Host  "publish package done: root@10.20.20.19:/var/lib/jenkins/workspace/publish/SDK/OrbbecSDK/$sw_version/C&C++/$sname"
# Remove-Item -Recurse -Force install