#ifndef TJSON_H
#define TJSON_H

#include <stddef.h>
#include "winTypes.h"

/*--------------------------------------------------------------------------------
   TJSON — one strong JSON parser, engine-agnostic, reused across the codebase.

   Decode parses a JSON text into a Son/Next node tree in ZERO string copies: every
   node's `text` is a VIEW (pointer + length) into the SOURCE buffer, so the source
   MUST OUTLIVE the returned tree. Nodes are pool-allocated; the destructor CASCADES
   (delete the root -> the entire tree frees itself). Navigate via the public son/next
   links, or the Child()/Elem() lookups. Malformed input yields a best-effort partial
   tree, never a crash (a permissive service parser).

   Node kinds (`type`): '{' object, '[' array, '"' string, '#' number/literal. An
   object's son-chain is its members: each member node carries the key in text/len and
   its VALUE as that member's own son. An array's son-chain is its elements.
  --------------------------------------------------------------------------------*/
class TJSON
{
 public:
   // Parse [text, text+len) -> root node; delete it to free the whole tree. NULL on empty.
   static TJSON *Decode(LPCSTR text, size_t len);

   ~TJSON(void);

   TJSON *son,  // first child (object member / array element / a member's value)
         *next; // next sibling in the chain

   LPCSTR text; // token view into the SOURCE (NOT NUL-terminated) — key name or scalar text
   size_t len;  // token length
   char   type; // '{' '[' '"' '#'

   // Object member VALUE by NUL-terminated key (this must be an object); NULL if absent.
   const TJSON *Child(LPCSTR key) const;

   // Array element by index (this must be an array); NULL if out of range.
   const TJSON *Elem(int index) const;

   // Scalar node -> integer (base-10, optional sign); def if not a number.
   long Int(long def) const;

 private:
   TJSON(void);
   TJSON(const TJSON &)            = delete;
   TJSON &operator=(const TJSON &) = delete;

   friend struct TJParse; // the recursive-descent parser (defined privately in json.cpp)
};

#endif // TJSON_H
