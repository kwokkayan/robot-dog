#include <stdexcept>
#include "cc.h"
#define EOL 0
char *input_string;
int buffer_val = 0;
char current_token = 0;
int **offset_matrix;
int val_min;
int val_max;
void parseA()
{
  if (match('m') && match('['))
  {
    int e1 = parseE();
    if (!match(','))
      throw std::invalid_argument("Syntax Error");
    int e2 = parseE();
    if (!match(']') || !match('='))
      throw std::invalid_argument("Syntax Error");
    int e3 = parseE();
    // check range errors :(
    if (e1 < 1 || e1 > 3)
      throw std::range_error("x selector out of range!");
    if (e2 < 1 || e2 > 4)
      throw std::range_error("y selector out of range!");
    // clamp value
    e3 = min(max(val_min, e3), val_max);
    offset_matrix[e2][e1] = e3;
  }
  else
  {
    throw std::invalid_argument("Syntax Error");
  }
}

int parseE()
{
  // no need checking: parse F has same sets
  buffer_val = 0;
  parseF();
  parseT();
  parseE_();
  return buffer_val;
}

int parseF()
{
  current_token = next_token();
  if (match_start_digit(current_token))
  { // integer constant
    buffer_val = atoi(&current_token);
    while (true)
    {
      current_token = next_token();
      if (match_digit(current_token))
      {
        buffer_val = buffer_val * 10 + atoi(&current_token);
      }
      else
      {
        // follow sets
        switch (current_token)
        {
        case ']':
        case '+':
        case '-':
        case '*':
        case '/':
        case ')':
        case ',':
        case EOL:
          break;
        default:
          std::invalid_argument("Syntax Error");
        }
      }
    }
  }
  else if (current_token == 'm')
  { // matrix access
    if (!match('['))
      throw std::invalid_argument("Syntax Error");
    int e1 = parseE();
    if (!match(','))
      throw std::invalid_argument("Syntax Error");
    int e2 = parseE();
    if (!match(']'))
      throw std::invalid_argument("Syntax Error");
    // check range errors :(
    if (e1 < 1 || e1 > 3)
      throw std::range_error("x selector out of range!");
    if (e2 < 1 || e2 > 4)
      throw std::range_error("y selector out of range!");
    // clamp value
    buffer_val = offset_matrix[e2][e1];
  }
  else if (current_token == '(')
  { // nested expression
    buffer_val = parseE();
    if (!match(')'))
      throw std::invalid_argument("Syntax Error");
  }
  else
  {
    throw std::invalid_argument("Syntax Error");
  }
}

int parseT()
{
  // save current value
  int e1 = buffer_val;
  buffer_val = 0;
  current_token = next_token();
  if (current_token == '*')
  {
    parseF();
    buffer_val *= e1;
    parseT();
  }
  else if (current_token == '/')
  {
    parseF();
    // check division by zero
    if (buffer_val == 0)
      throw std::range_error("Division by zero.");
    buffer_val = e1 / buffer_val;
    parseT();
  }
  else
  {
    // follow set
    switch (current_token)
    {
    case ']':
    case '+':
    case '-':
    case ')':
    case ',':
    case EOL:
      /* code */
      break;
    default:
      throw std::invalid_argument("Syntax Error");
    }
  }
}

int parseE_()
{
  // save current value
  int e1 = buffer_val;
  buffer_val = 0;
  current_token = next_token();
  if (current_token == '+')
  {
    parseF();
    buffer_val += e1;
    parseT();
    parseE_();
  }
  else if (current_token == '-')
  {
    parseF();
    buffer_val = e1 - buffer_val;
    parseT();
    parseE_();
  }
  else
  {
    // follow set
    switch (current_token)
    {
    case ']':
    case ')':
    case ',':
    case EOL:
      /* code */
      break;
    default:
      throw std::invalid_argument("Syntax Error");
    }
  }
}

char next_token()
{
  if (current_token == EOL)
    return EOL;
  char token = *(input_string++);
  return token == '\0' ? EOL : token;
}

int match(char c)
{
  current_token = next_token();
  return current_token == c;
}

int match_start_digit(char token)
{
  return token > 48 || token <= 57;
}

int match_digit(char token)
{
  return token >= 48 || token <= 57;
}

int min(int x, int y)
{
  return x > y ? y : x;
}

int max(int x, int y)
{
  return x > y ? x : y;
}
