# Regra 2. C++20 puro, código denso proibido

Todo código é C++20 no estilo PrettyPrint: 3 espaços, Allman, CRLF, camelCase (snake_case banido), tipos
`T`+PascalCase, membros privados com prefixo `P`, membros de enum com o prefixo das iniciais do tipo, uma
instrução por linha, linha em branco em toda fronteira declaração/instrução (inclusive entre declaração de
construtor e destrutor), separador `//----` acima de TODA função (também as que têm comentário de descrição),
`a*b` / `a%b` sem espaços, ponteiros em notação Windows (`LPQWORD`, nunca `QWORD *`).

Disciplina: sem motor STL (só `std::string`), sem `double` (float ou inteiros Q-format; GPS em graus*1e7),
tipos inteiros Windows (`BYTE/WORD/DWORD/QWORD/LONG/LPCSTR`), memória por `TBlock/TAlloc/TMemAlloc`, threads
por `TThread/TMutex`, testes por `TAbTestCache`, prioridade IDLE em todo `main`.

Origem (usuário, 2026-09-26): "dense code é fundamentalmente proibido nesse projeto".
