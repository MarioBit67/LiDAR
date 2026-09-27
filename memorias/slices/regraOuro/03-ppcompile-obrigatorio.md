# Regra 3. Toda compilação passa pelo ppCompile original; ppCheck -deep antes de qualquer build

A compilação é executada obrigatoriamente por
`E:\Projetos\Claude\shared\tools\ppCheck\build\Release\ppCompile.exe` - host (MSBuild `CLToolExe`) e mobile
(modo launcher sobre o clang do NDK). O usuário aceitou que o ppCheck atualize o próprio `ppCheckCache.json`
naquela pasta.

- Antes de qualquer build: `ppCheck.exe -deep <arquivos>` (detecção, não grava cache). Não verde = PARA.
- O `sed -i` do Git Bash remove CR: renormalize com `sed -i 's/\r*$/\r/'` logo antes de checar e filtre a saída.
- Builds em background e IDLE (`nice -n 19`), gerador VS18.

Origem (usuário, 2026-09-26): "a compilação deve obrigatoriamente ser executada por ...ppCompile.exe, que
cuida de regras de ortografia"; depois escolheu "usar o caminho original".
