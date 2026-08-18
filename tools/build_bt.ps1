<#
.SYNOPSIS
  AIP_DCS BehaviorTree DLL 을 빌드하고 DogFightEnv\Release 로 배치한다.

.DESCRIPTION
  구성은 Debug|x64 다. Release 를 쓰지 마라:
    - vendor 가 배포한 DLL 자체가 Debug 빌드다. JSBSimAIPLib.dll / AIP_BASE*.dll 의 import 를
      보면 MSVCP140D.dll / VCRUNTIME140D.dll / VCRUNTIME140_1D.dll / ucrtbased.dll,
      즉 디버그 CRT 를 요구한다. Release CRT 로 만든 DLL 을 그 호스트에 섞지 마라.
    - 디버그 CRT 는 재배포 불가라 Visual Studio(또는 VS Build Tools 의 MSVC v143) 설치본에만 있다.
      MSVC 를 설치하면 System32 에 자동 배치되므로 PATH 를 손댈 필요는 없다.
      (오히려 VS 의 debug_nonredist / Windows Kits 의 ucrt 폴더를 PATH 앞에 붙이면
       System32 의 ucrtbase.dll 을 가려 엉뚱한 크래시가 난다.)

  빌드 전에 tools\fix_host_project.ps1 을 먼저 돌려라. vcxproj 에 노드가 등록돼 있지 않으면
  **에러 없이** 그 노드가 빠진 DLL 이 나온다.

  소스 동기화 (2026-08-04 추가, 기본 동작)
  ----------------------------------------
  vcxproj 는 팀 저장소가 아니라 <HostRoot>\BehaviorTree 를 컴파일한다. 즉 팀 트리를
  고쳐도 그쪽으로 복사하지 않으면 **옛 소스가 그대로 빌드된다.**

  실제로 이 때문에 xml_parsing.cpp 의 수정(자식 노드 연결을 이름 문자열이 아니라
  타입으로 판정)이 반영되지 않아, Rule.xml 의 모든 제어 노드에 name 별칭이 붙어 있는 탓에
  **트리의 자식이 하나도 연결되지 않은 DLL** 이 만들어졌다. BT 는 매 tick SUCCESS 를
  돌려주면서 아무 노드도 실행하지 않았고(BFM=NONE 1468/1468, VP=(0,0,0)), 에러가 한 줄도
  나지 않아 오랫동안 발견되지 않았다.

  그래서 이 스크립트는 빌드 전에 팀 트리를 호스트로 동기화하고, **무엇이 낡아 있었는지
  파일 이름까지 출력한다.** -NoSync 로 끌 수 있지만 권장하지 않는다.

  주의: BT_Content 만이 아니라 저장소 루트의 xml_parsing.cpp / behavior_tree.cpp 같은
  라이브러리 소스도 함께 봐야 한다. 예전 절차는 BT_Content 만 복사해서 이 결함을 놓쳤다.

  ATK 분기 (2026-08-13)
  ---------------------
  이 저장소는 공격형 실험 브랜치(ATK)다. 원본 Behaviortree 와 A/B 비교를 하려면
  두 빌드가 서로를 덮어쓰지 않아야 하므로, 기본값이 전부 ATK 전용 경로로 바뀌어 있다.

    호스트 트리 : C:\AIP_LIB\AIP_DCS_ATK     (원본은 C:\AIP_LIB\AIP_DCS)
    빌드 산출   : C:\AIP_LIB\bin_atk\...     (원본은 C:\AIP_LIB\bin\...)
    DLL 이름    : AIP_STIL_ATK.dll           (원본은 AIP_STIL.dll)
    Rule XML    : Rule_ATK.xml               (원본은 Rule.xml)

  Release 루트에 두 DLL 과 두 XML 이 공존하고, 실행 쪽에서 BT_RULE_XML 로 어느 XML 을
  읽을지 고른다(CPPBehaviorTree.cpp 의 init() 참조). 그래서 원본을 건드리지 않고
  ownship=ATK / target=원본 같은 대전을 그대로 돌릴 수 있다.

.PARAMETER HostRoot
  AIP_DCS 폴더(.sln 이 있는 곳). 환경변수 AIP_DCS_ROOT 로도 지정 가능.
  주의: AIP_DCS_ROOT 가 원본을 가리키도록 설정돼 있으면 ATK 소스가 원본 호스트로
  동기화된다. ATK 에서는 환경변수를 두지 말고 기본값을 쓰는 편이 안전하다.

.PARAMETER BinName
  산출물 루트 폴더 이름. vcxproj 의 OutDir 과 반드시 같아야 한다.

.PARAMETER RuleXmlName
  Release 루트에 배치할 XML 파일명. 저장소 안에서는 원본과 diff 가 되도록
  계속 Rule.xml 이고, 배치할 때만 이 이름으로 바뀐다.

.PARAMETER NoSync
  팀 트리 -> 호스트 소스 동기화를 건너뛴다. 낡은 소스가 빌드될 수 있다.

.PARAMETER ReleaseDir
  DogFightEnv\Release 경로. -Deploy 일 때만 쓴다.

.PARAMETER TeamName
  배치할 DLL 이름에 쓰인다(AIP_<TeamName>.dll). vendor 의 AIP_BASE*.dll 은 절대 덮어쓰지 않는다.

.EXAMPLE
  .\tools\build_bt.ps1
  .\tools\build_bt.ps1 -Deploy
#>
param(
    [string]$HostRoot    = $(if ($env:AIP_DCS_ROOT) { $env:AIP_DCS_ROOT } else { "C:\AIP_LIB\AIP_DCS_ATK" }),
    [string]$ReleaseDir  = "C:\AIP_LIB\DogFightEnv\Release",
    [string]$Config      = "Debug",
    [string]$Platform    = "x64",
    [string]$TeamName    = "STIL_ATK",
    [string]$BinName     = "bin_atk",
    # [2026-08-18] 07f149f 가 저장소의 Rule.xml 을 Rule_STIL_ATK.xml 로 rename 하고
    # CPPBehaviorTree.cpp 의 기본값도 "./Rule_STIL_ATK.xml" 로 바꿨다. 배포 이름을
    # 그 기본값과 같게 두면 BT_RULE_XML 을 주지 않아도 올바른 XML 을 읽는다.
    [string]$RuleXmlName = "Rule_STIL_ATK.xml",
    # 저장소/호스트 트리 안에서의 XML 파일 이름.
    [string]$RuleXmlSource = "Rule_STIL_ATK.xml",
    [switch]$Deploy,
    # A/B 비교용. 각도 wrap 보정만 끈 'before' DLL 을 만든다.
    # BT_Content/AngleUtil.h 의 PM_DISABLE_WRAP_FIX 설명 참조.
    # 이 스위치로 만든 DLL 은 제출/실전에 쓰지 말 것.
    [switch]$DisableWrapFix,
    # 팀 트리 -> 호스트 소스 동기화를 건너뛴다. 낡은 소스가 빌드될 수 있어 권장하지 않는다.
    [switch]$NoSync
)

$ErrorActionPreference = "Stop"

$sln = Join-Path $HostRoot "AIP_DCS.sln"
if (-not (Test-Path $sln)) { throw ".sln 을 찾을 수 없다: $sln" }

# ---- 소스 동기화 (팀 트리 -> 호스트 빌드 트리) ----------------------
# vcxproj 가 컴파일하는 것은 <HostRoot>\BehaviorTree 다. 팀 트리를 고쳐도
# 여기에 복사하지 않으면 옛 소스가 조용히 빌드된다(.SYNOPSIS 의 사고 사례 참조).
$TeamRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$HostBT   = Join-Path $HostRoot "BehaviorTree"

function Sync-BehaviorTreeSources {
    param([string]$From, [string]$To)

    if (-not (Test-Path $To)) { throw "호스트 BehaviorTree 폴더가 없다: $To" }

    # 빌드에 들어가지 않는 것만 제외한다. 소스/헤더는 전부 대상이다.
    $skipDirs = @('.git', '.github', 'tools', 'tests')
    $skipNames = @('.DS_Store', '.gitignore')

    $files = Get-ChildItem -Path $From -Recurse -File | Where-Object {
        $rel = $_.FullName.Substring($From.Length).TrimStart('\')
        $top = ($rel -split '\\')[0]
        ($skipDirs -notcontains $top) -and
        ($skipNames -notcontains $_.Name) -and
        ($_.Extension -notin @('.md'))
    }

    $copied = @()
    foreach ($f in $files) {
        $rel = $f.FullName.Substring($From.Length).TrimStart('\')
        $dst = Join-Path $To $rel
        $needCopy = $true
        if (Test-Path $dst) {
            # 해시 비교. 타임스탬프는 복사/체크아웃으로 쉽게 흔들려 신뢰할 수 없다.
            $needCopy = (Get-FileHash $f.FullName -Algorithm SHA256).Hash -ne
                        (Get-FileHash $dst      -Algorithm SHA256).Hash
        }
        if ($needCopy) {
            $dstDir = Split-Path $dst -Parent
            if (-not (Test-Path $dstDir)) { New-Item -ItemType Directory -Force -Path $dstDir | Out-Null }
            Copy-Item $f.FullName $dst -Force
            $copied += $rel
        }
    }
    return $copied
}

if ($NoSync) {
    Write-Host "[sync] 건너뜀 (-NoSync). 호스트의 낡은 소스가 빌드될 수 있다." -ForegroundColor Yellow
} else {
    Write-Host "[sync] $TeamRoot  ->  $HostBT"
    $changed = Sync-BehaviorTreeSources -From $TeamRoot -To $HostBT
    if ($changed.Count -eq 0) {
        Write-Host "[sync] 이미 최신 (변경 없음)"
    } else {
        Write-Host "[sync] 낡아 있던 파일 $($changed.Count)개를 갱신했다:" -ForegroundColor Yellow
        $changed | Select-Object -First 20 | ForEach-Object { Write-Host "         $_" }
        if ($changed.Count -gt 20) { Write-Host "         ... 외 $($changed.Count - 20)개" }
    }

    # 호스트에만 있는 소스는 vcxproj 가 컴파일할 수 있으므로 알려 준다(삭제하지는 않는다).
    $hostOnly = Get-ChildItem -Path $HostBT -Recurse -File -Include *.cpp, *.h |
        Where-Object {
            $rel = $_.FullName.Substring($HostBT.Length).TrimStart('\')
            -not (Test-Path (Join-Path $TeamRoot $rel))
        } | ForEach-Object { $_.FullName.Substring($HostBT.Length).TrimStart('\') }
    if ($hostOnly) {
        Write-Host "[sync] 호스트에만 있는 소스 $($hostOnly.Count)개 (팀 트리에 없음, 그대로 둔다):" -ForegroundColor Yellow
        $hostOnly | Select-Object -First 10 | ForEach-Object { Write-Host "         $_" }
    }
}

# ---- MSBuild 찾기 -------------------------------------------------
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) {
    throw "vswhere 가 없다. VS Build Tools 를 설치하라:`n" +
          "  winget install Microsoft.VisualStudio.2022.BuildTools 또는 https://aka.ms/vs/17/release/vs_BuildTools.exe`n" +
          "  필요 컴포넌트: Microsoft.VisualStudio.Component.VC.Tools.x86.x64, Windows 11 SDK"
}
$msbuild = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild `
                      -find "MSBuild\**\Bin\MSBuild.exe" | Select-Object -First 1
if (-not $msbuild) { throw "MSBuild.exe 를 찾지 못했다. VC 워크로드가 설치돼 있는지 확인하라." }
Write-Host "MSBuild: $msbuild"

# ---- 빌드 ---------------------------------------------------------
$suffix = if ($DisableWrapFix) { "_nowrapfix" } else { "" }
$log = Join-Path $PSScriptRoot "msbuild_$Config`_$Platform$suffix.log"
Write-Host "빌드: $Config|$Platform"

# PM_DISABLE_WRAP_FIX 는 전처리기 정의로만 넣는다. 소스를 건드리지 않으므로
# 두 빌드의 차이가 이 매크로 하나뿐임이 보장된다.
#
# /p:PreprocessorDefinitions 로는 전달되지 않는다. vcxproj 에서 그 값은 property 가
# 아니라 ClCompile 의 item metadata 라 전역 property 로 덮이지 않기 때문이다.
# cl.exe 가 직접 읽는 CL 환경변수를 쓴다(옵션을 명령줄 앞에 붙여 준다).
$prevCL = $env:CL
if ($DisableWrapFix) {
    Write-Host "  [A/B] PM_DISABLE_WRAP_FIX 켜짐 - wrap 보정이 꺼진 'before' 빌드다."
    Write-Host "        제출/실전에 쓰지 말 것."
    $env:CL = ("/DPM_DISABLE_WRAP_FIX " + $env:CL).Trim()
}

# 매크로가 바뀌어도 증분 빌드는 이를 감지하지 못한다. 항상 Rebuild 해야
# 이전 빌드의 .obj 가 섞이지 않는다.
$msbuildTarget = if ($DisableWrapFix) { "Rebuild" } else { "Build" }
try {
    & $msbuild $sln /t:$msbuildTarget /p:Configuration=$Config /p:Platform=$Platform /m /nologo `
        /v:minimal /flp:"logfile=$log;verbosity=normal" 2>&1 | Tee-Object -Variable out | Out-Host
} finally {
    $env:CL = $prevCL
}
$code = $LASTEXITCODE

$errors   = @($out | Select-String -Pattern ' error [A-Z]+\d+')
$warnings = @($out | Select-String -Pattern ' warning [A-Z]+\d+')
Write-Host ""
Write-Host ("결과: exit={0}, error={1}, warning={2}" -f $code, $errors.Count, $warnings.Count)
Write-Host "전체 로그: $log"

if ($errors.Count -gt 0) {
    Write-Host "`n--- 첫 컴파일 에러 5개 ---"
    $errors | Select-Object -First 5 | ForEach-Object { "  " + ($_.Line -replace '\s*\[.*\]$','').Trim() }
}
if ($code -ne 0) { exit $code }

# ---- 산출물 -------------------------------------------------------
# vcxproj 의 OutDir 은 $(SolutionDir)..\$BinName\$(Configuration).$(Platform)\ 이다.
# 여기서 다른 곳을 보면 "DLL 이 없다" 로 끝나거나, 더 나쁘게는 원본 트리의 DLL 을
# 집어 배치한다. 둘은 반드시 같이 움직여야 한다.
$outDir = Join-Path (Split-Path $HostRoot -Parent) ("{0}\{1}.{2}" -f $BinName, $Config, $Platform)
$dll = Get-ChildItem $outDir -Filter *.dll -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending
if (-not $dll) { Write-Host "!! DLL 이 없다: $outDir"; exit 1 }
Write-Host "`n산출 DLL:"
$dll | ForEach-Object { "  {0}  {1:N0} bytes  {2}" -f $_.FullName, $_.Length, $_.LastWriteTime }

# ---- 배치 ---------------------------------------------------------
if (-not $Deploy) {
    # -Deploy 없이 끝내면 Release 루트의 Rule.xml 은 그대로다. XML 만 고친 경우
    # 빌드할 이유가 없어 이 스크립트조차 안 돌리게 되고, DLL 은 옛 XML 을 계속 읽는다.
    # 파싱은 성공하므로 에러 없이 노드 구성만 달라진다 — 조용해서 추적이 어렵다.
    $rtXml = Join-Path $ReleaseDir $RuleXmlName
    $tmXml = Join-Path $TeamRoot   $RuleXmlSource
    if ((Test-Path $rtXml) -and (Test-Path $tmXml)) {
        $hr = (Get-FileHash $rtXml -Algorithm SHA256).Hash
        $ht = (Get-FileHash $tmXml -Algorithm SHA256).Hash
        if ($hr -ne $ht) {
            Write-Host ""
            Write-Warning "Release 루트의 Rule.xml 이 팀 트리와 다르다. DLL 은 옛 XML 을 읽는다."
            Write-Host  "         팀     : $tmXml"
            Write-Host  "         런타임 : $rtXml"
            Write-Host  "         맞추려면: .	ools\sync_rule_xml.ps1 -Apply"
            Write-Host  "         (C++ 도 고쳤다면 이 스크립트를 -Deploy 로 다시 돌려라)"
        }
    }
}

if ($Deploy) {
    if (-not (Test-Path $ReleaseDir)) { throw "ReleaseDir 이 없다: $ReleaseDir" }
    $target  = $dll | Select-Object -First 1
    $teamDll = Join-Path $ReleaseDir "AIP_$TeamName.dll"
    $teamXml = Join-Path $ReleaseDir $RuleXmlName

    foreach ($p in @($teamDll, $teamXml)) {
        if (Test-Path $p) {
            $stamp = (Get-Item $p).LastWriteTime.ToString('yyyyMMddHHmmss')
            Copy-Item $p "$p.bak_$stamp"
            Write-Host "  기존 파일 백업: $p.bak_$stamp"
        }
    }

    # [2026-08-18] 대상 파일을 먼저 지운다. Windows 는 대소문자를 구분하지 않지만
    # 보존은 하므로, 예전 이름(예: Rule_stil.xml)이 남아 있으면 Copy-Item 이 내용만
    # 덮어쓰고 이름은 옛 케이싱으로 유지한다. 런타임은 열리지만 규정 §9 제출물
    # 이름이 틀리게 되고, ls 로 봐야만 드러나 조용히 넘어간다.
    foreach ($p in @($teamDll, $teamXml)) { Remove-Item $p -Force -ErrorAction SilentlyContinue }

    Write-Host "`n[deploy] -> $ReleaseDir"
    Copy-Item $target.FullName -Destination $teamDll -Force
    Copy-Item (Join-Path $HostRoot "BehaviorTree\$RuleXmlSource") -Destination $teamXml -Force
    Write-Host "  $($target.Name) -> AIP_$TeamName.dll"
    Write-Host "  $RuleXmlSource -> $RuleXmlName"
    Write-Host ""
    Write-Host "  이 DLL 의 기본 XML 은 ./$RuleXmlName 이고(CPPBehaviorTree.cpp),"
    Write-Host "  배포 이름을 그와 같게 맞췄으므로 BT_RULE_XML 없이도 올바른 XML 을 읽는다."
    Write-Host "  다른 XML 로 A/B 를 할 때만 환경변수를 준다:"
    Write-Host "        `$env:BT_RULE_XML = `"./$RuleXmlName`""
}
