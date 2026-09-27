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

- Não regenerar o CMake (`cmake -S . -B build`) nem mexer no CMakeLists sem necessidade real: a solução fica
  aberta no Visual Studio do usuário, e cada regeneração recarrega os projetos e abre a janela de saída
  (usuário, 2026-09-27: "as atualizações do CMake não precisam ser validadas por mim"). Fonte nova no core:
  agrupe as mudanças do CMakeLists e regenere uma vez só. No dia a dia, só `cmake --build`.
