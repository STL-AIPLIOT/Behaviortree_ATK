<#
.SYNOPSIS
  Behaviortree_ATK -> Behaviortree 소스 동기화. 두 트리를 코드 동등하게 유지한다.

.DESCRIPTION
  두 트리는 같은 코드를 쓰고 배포 이름만 다르다:

      Behaviortree      AIP_STIL.dll     + Rule_STIL.xml
      Behaviortree_ATK  AIP_STIL_ATK.dll + Rule_STIL_ATK.xml

  그래서 소스 차이는 CPPBehaviorTree.cpp 의 기본 XML 문자열 한 줄뿐이어야 한다.
  문제는 단순 복사가 그 한 줄까지 ATK 값으로 덮어버린다는 것이다. 그대로 빌드하면
  AIP_STIL.dll 이 Rule_STIL_ATK.xml 을 읽는다 - 파싱은 성공하므로 에러 없이
  A/B 비교만 조용히 무의미해진다. 실제로 여러 번 손으로 되돌려야 했다.

  이 스크립트는 복사 후 그 한 줄을 자동으로 되돌린다. XML 파일도 이름을 바꿔
  옮기므로 Behaviortree 쪽에 Rule_STIL_ATK.xml 이 섞이지 않는다.

  제외 대상:
    .git .github tools .vscode  - 저장소/도구 (tools 는 두 트리의 기본값이 다르다)
    *.md                        - 문서
    tests\build                 - 빌드 산출물

.PARAMETER Check
  복사하지 않고 차이만 보고한다. CI 나 커밋 전 확인용.
#>
param(
    [string]$Src = (Join-Path (Split-Path $PSScriptRoot -Parent) ""),
    [string]$Dst = (Join-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) "Behaviortree"),
    [switch]$Check
)

$ErrorActionPreference = "Stop"
$Src = (Resolve-Path $Src).Path
$Dst = (Resolve-Path $Dst).Path

if ($Src -eq $Dst) { throw "원본과 대상이 같다: $Src" }
Write-Host "[sync] $Src"
Write-Host "    -> $Dst"

$skipTop = @('.git', '.github', 'tools', '.vscode')
$skipExt = @('.md')

# ---- 1. 소스 복사 -------------------------------------------------------
$files = Get-ChildItem -Path $Src -Recurse -File | Where-Object {
    $rel = $_.FullName.Substring($Src.Length).TrimStart('\')
    $top = ($rel -split '\\')[0]
    ($skipTop -notcontains $top) -and
    ($skipExt -notcontains $_.Extension) -and
    ($rel -notlike 'tests\build\*') -and
    ($_.Extension -ne '.xml')          # XML 은 아래에서 이름을 바꿔 따로 옮긴다
}

# 의도된 차이를 원본 내용에 미리 적용해 비교한다. 그러지 않으면
# CPPBehaviorTree.cpp 가 영원히 "불일치" 로 잡힌다.
function Convert-ToDstIdentity([string]$text) {
    $text = $text.Replace('rulePath = "./Rule_STIL_ATK.xml";', 'rulePath = "./Rule_STIL.xml";')
    $text = $text.Replace('없으면 팀 고유 이름인 "./Rule_STIL_ATK.xml" 이다.',
                          '없으면 팀 고유 이름인 "./Rule_STIL.xml" 이다. (ATK 트리는 Rule_STIL_ATK.xml)')
    return $text
}

$identityFiles = @('CPPBehaviorTree.cpp')

$changed = @()
$identityFixed = $false
foreach ($f in $files) {
    $rel = $f.FullName.Substring($Src.Length).TrimStart('\')
    $d = Join-Path $Dst $rel

    if ($identityFiles -contains $rel) {
        # 변환본과 대상을 비교한다.
        $srcText = [Text.Encoding]::UTF8.GetString([IO.File]::ReadAllBytes($f.FullName))
        $want = Convert-ToDstIdentity $srcText
        if ($want -ne $srcText) { $identityFixed = $true }
        $have = if (Test-Path $d) { [Text.Encoding]::UTF8.GetString([IO.File]::ReadAllBytes($d)) } else { $null }
        if ($have -ne $want) {
            $changed += $rel
            if (-not $Check) {
                [IO.File]::WriteAllBytes($d, [Text.Encoding]::UTF8.GetBytes($want))
            }
        }
        continue
    }

    $need = $true
    if (Test-Path $d) {
        $need = (Get-FileHash $f.FullName -Algorithm SHA256).Hash -ne
                (Get-FileHash $d          -Algorithm SHA256).Hash
    }
    if ($need) {
        $changed += $rel
        if (-not $Check) {
            $dd = Split-Path $d -Parent
            if (-not (Test-Path $dd)) { New-Item -ItemType Directory -Force -Path $dd | Out-Null }
            Copy-Item $f.FullName $d -Force
        }
    }
}

# ---- 2. Rule XML : 이름을 바꿔 옮긴다 -----------------------------------
$srcXml = Join-Path $Src "Rule_STIL_ATK.xml"
$dstXml = Join-Path $Dst "Rule_STIL.xml"
$xmlChanged = $false
if (Test-Path $srcXml) {
    $xmlChanged = -not (Test-Path $dstXml) -or
        ((Get-FileHash $srcXml -Algorithm SHA256).Hash -ne (Get-FileHash $dstXml -Algorithm SHA256).Hash)
    if ($xmlChanged -and -not $Check) { Copy-Item $srcXml $dstXml -Force }
}

# ---- 4. 보고 ------------------------------------------------------------
$verb = if ($Check) { "차이" } else { "갱신" }
Write-Host ("[sync] 소스 {0} {1}개" -f $verb, $changed.Count)
$changed | Select-Object -First 8 | ForEach-Object { Write-Host "         $_" }
if ($changed.Count -gt 8) { Write-Host ("         ... 외 {0}개" -f ($changed.Count - 8)) }
if ($xmlChanged)   { Write-Host "[sync] Rule_STIL_ATK.xml -> Rule_STIL.xml" }
if ($identityFixed) { Write-Host "[sync] CPPBehaviorTree.cpp 의 기본 XML 문자열을 Rule_STIL.xml 로 복구" -ForegroundColor Yellow }

if ($Check) {
    if ($changed.Count -eq 0 -and -not $xmlChanged) {
        Write-Host "[sync] 동기화됨" -ForegroundColor Green
        exit 0
    }
    Write-Host "[sync] 동기화 필요 - -Check 없이 다시 실행하라" -ForegroundColor Yellow
    exit 1
}

# 대상에만 있는 소스는 알려만 준다(삭제하지 않는다).
$only = Get-ChildItem -Path $Dst -Recurse -File -Include *.cpp, *.h | Where-Object {
    $rel = $_.FullName.Substring($Dst.Length).TrimStart('\')
    $top = ($rel -split '\\')[0]
    ($skipTop -notcontains $top) -and ($rel -notlike 'tests\build\*') -and
    -not (Test-Path (Join-Path $Src $rel))
} | ForEach-Object { $_.FullName.Substring($Dst.Length).TrimStart('\') }
if ($only) {
    Write-Host "[sync] 대상에만 있는 소스 (그대로 둔다):"
    $only | ForEach-Object { Write-Host "         $_" }
}
