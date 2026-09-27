/* json.cpp — TJSON, the one strong JSON parser (see json.h). Recursive-descent over a
   zero-copy cursor: string/scalar tokens are VIEWS into the source, nodes are pool-backed
   scalar allocations, and ~TJSON cascades the Son/Next tree. No libc alloc, no fp. */

#ifndef SHARED_LIBRARY_BUILD
#define SHARED_LIBRARY_BUILD
#endif

#include "json.h"
#include <string.h>        // memchr / memcmp / strlen
#include "libDiscipline.h" // allocator/fp64 poison — last include

//---------------------------------------------------------------------------------------------------

TJSON::TJSON(void) : son(NULL), next(NULL), text(NULL), len(0u), type(0)
{
}

TJSON::~TJSON(void)
{
   TJSON *n, *nx;

   delete son; // recurse into this node's subtree

   for (n = next; n; n = nx) // walk the sibling chain iteratively (no deep recursion)
   {
      nx = n->next;
      n->next = NULL;
      delete n;
   }
}

/*--------------------------------------------------------------------------------
   TJParse — the private recursive-descent parser (friend of TJSON). Holds the read
   cursor [p, end); each token method advances p and returns a freshly built node.
  --------------------------------------------------------------------------------*/
struct TJParse
{
   LPCSTR p, end;

   void   skipWs(void);
   TJSON *Value(void);
   TJSON *Object(void);
   TJSON *Array(void);
   TJSON *strTok(void);
   TJSON *numTok(void);
};

//--------------------------------------------------------------------------------
void TJParse::skipWs(void)
{
   while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r'))
      p++;
}

//--------------------------------------------------------------------------------
TJSON *TJParse::strTok(void)
{
   LPCSTR s, q, close, hit, bs;
   TJSON *j;

   s = p + 1; // past the opening quote
   q = s;
   close = NULL;

   for (;;) // memchr to the next '"', accept it only if an EVEN run of '\' precedes it
   {
      hit = (LPCSTR)memchr(q, '"', (size_t)(end - q));
      if (!hit)
         break;
      bs = hit;
      while (bs > s && bs[-1] == '\\')
         bs--;
      if ((((size_t)(hit - bs)) & 1u) == 0u)
      {
         close = hit;
         break;
      }
      q = hit + 1;
   }

   j = new TJSON;
   j->type = '"';
   j->text = s;
   j->len = (size_t)((close ? close : end) - s);
   p = close ? close + 1 : end;
   return j;
}

//--------------------------------------------------------------------------------
TJSON *TJParse::numTok(void)
{
   LPCSTR s;
   TJSON *j;

   s = p;

   while (p < end && *p != ',' && *p != ']' && *p != '}' &&
          *p != ' ' && *p != '\t' && *p != '\n' && *p != '\r')
      p++;

   j = new TJSON;
   j->type = '#';
   j->text = s;
   j->len = (size_t)(p - s);
   return j;
}

//--------------------------------------------------------------------------------
TJSON *TJParse::Object(void)
{
   TJSON *j, *m, **tail;

   j = new TJSON;
   j->type = '{';
   tail = &j->son;
   p++; // consume '{'
   skipWs();

   while (p < end && *p != '}')
   {
      if (*p != '"')
         break; // malformed member — bail best-effort
      m = strTok();
      skipWs();
      if (p < end && *p == ':')
      {
         p++;
         skipWs();
         m->son = Value(); // the member's value hangs off its son
      }
      *tail = m;
      tail = &m->next;
      skipWs();
      if (p < end && *p == ',')
      {
         p++;
         skipWs();
      }
   }

   if (p < end)
      p++; // consume '}'
   return j;
}

//--------------------------------------------------------------------------------
TJSON *TJParse::Array(void)
{
   TJSON *j, **tail;

   j = new TJSON;
   j->type = '[';
   tail = &j->son;
   p++; // consume '['
   skipWs();

   while (p < end && *p != ']')
   {
      *tail = Value();
      if (*tail)
         tail = &(*tail)->next;
      skipWs();
      if (p < end && *p == ',')
      {
         p++;
         skipWs();
      }
   }

   if (p < end)
      p++; // consume ']'
   return j;
}

//--------------------------------------------------------------------------------
TJSON *TJParse::Value(void)
{
   skipWs();

   if (p >= end)
      return NULL;
   if (*p == '{')
      return Object();
   if (*p == '[')
      return Array();
   if (*p == '"')
      return strTok();
   return numTok();
}

//---------------------------------------------------------------------------------------------------

TJSON *TJSON::Decode(LPCSTR text, size_t len)
{
   TJParse js;

   if (!text || len == 0u)
      return NULL;
   js.p = text;
   js.end = text + len;
   return js.Value();
}

//--------------------------------------------------------------------------------
const TJSON *TJSON::Child(LPCSTR key) const
{
   const TJSON *m;
   size_t klen;

   if (!key)
      return NULL;
   klen = strlen(key);

   for (m = son; m; m = m->next)
      if (m->len == klen && memcmp(m->text, key, klen) == 0)
         return m->son; // the member's VALUE (the key lives in the member node's text/len)
   return NULL;
}

//--------------------------------------------------------------------------------
const TJSON *TJSON::Elem(int index) const
{
   const TJSON *e;
   int i;

   if (index < 0)
      return NULL;

   for (e = son, i = 0; e; e = e->next, i++)
      if (i == index)
         return e;
   return NULL;
}

//--------------------------------------------------------------------------------
long TJSON::Int(long def) const
{
   long   v;
   int    neg;
   LPCSTR q, stop;

   if (type != '#' || !text || len == 0u)
      return def;
   q = text;
   stop = text + len;
   neg = 0;

   if (*q == '-')
   {
      neg = 1;
      q++;
   }
   else if (*q == '+')
      q++;

   v = 0;
   while (q < stop && *q >= '0' && *q <= '9')
   {
      v = v*10 + (*q - '0');
      q++;
   }
   return neg ? -v : v;
}
