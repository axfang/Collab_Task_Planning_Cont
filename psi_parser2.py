from dataclasses import dataclass  # if python < 3.7
from typing import Optional, List
from operator import add, sub, mul, truediv
import re

@dataclass
class Node:
    symbol: str
    left: Optional['Node']
    right: Optional['Node']

    def is_leaf(self) -> bool:
        return self.left is None and self.right is None

@dataclass
class Tree:
    root: Node

    @classmethod
    def _tokenize(cls, text: str) -> List[str]:
        prev = ''
        tokenized = []
        for char in text:
            if prev.isdigit() and char.isdigit():
                tokenized.append(tokenized.pop() + char)
            else:
                tokenized.append(char)
            prev = char
        return tokenized

    @classmethod
    def build(cls, text: str) -> 'Tree':

        # stack: List[Node] = []
        # prev_char = 0
        # for char in cls._tokenize(text):
        #     if char.isalnum():
        #         val=1
        #         if prev_char == '!':
        #             # char = prev_char + char
        #             val = -1
        #         stack.append(Node(symbol=(char,val), left=None, right=None))
        #     if char == ')':
        #         right = stack.pop()
        #         op = stack.pop()
        #         left = stack.pop()
        #         stack.append(Node(symbol=op, left=left, right=right))

        #     elif char in '&|':
        #         stack.append(' ' + char + ' ')


        #     prev_char = char
        # return cls(root=stack.pop())

        operator_stack: List[str] = []
        operand_stack: List[Node] = []
        for char in cls._tokenize(text):
            # print(operator_stack, operand_stack)
            if char.isalnum():
                operand_stack.append(Node(symbol=char, left=None, right=None))
            elif char == ')':
                while len(operator_stack) > 0 and operator_stack[-1] != '(':
                    right = operand_stack.pop()
                    op = operator_stack.pop()
                    left = operand_stack.pop()
                    operand_stack.append(Node(symbol=op, left=left, right=right))
                operator_stack.pop()
            else:
                operator_stack.append(char)
        while len(operator_stack) > 0:
            right = operand_stack.pop()
            op = operator_stack.pop()
            left = operand_stack.pop()
            operand_stack.append(Node(symbol=op, left=left, right=right))
        return cls(root=operand_stack.pop())

    # def evaluate(self, node: Optional[Node] = None, op = None):
    #     node = node or self.root
    #     # print(node.is_leaf())
    #     if node.is_leaf():
    #         if '!' in node.symbol and len(node.symbol) > 1:
    #             return '!a.[' + str(node.symbol[1:]) + ']'
    #         else:
    #             return 'a.[' + str(node.symbol) + ']'
    #     else:
    #         return self.evaluate(node.left), self.evaluate(node.right)
            # return op(self.evaluate(node.left), self.evaluate(node.right))
    @classmethod
    def evaluate(self, ap, node: Optional[Node] = None, string=""):
            node = node or self.root
            if node.is_leaf():
                # if '!' in node.symbol and len(node.symbol) > 1:
                #     string += '!a.[' + str(node.symbol[1:]) + ']' 
                # else:
                num = node.symbol
            
                string += ap + '.[' + str(num) + ']'
                return string

            else:

                return '(' + self.evaluate(ap, node.left, string) + str(node.symbol) + self.evaluate(ap, node.right,string) + ')'


    @classmethod
    def outputLeafNodes(self, node: Optional[Node] = None, leaf_list = []):
        node = node or self.root
     
        # If node is leaf node,
        # print its data
        if node.is_leaf():
            leaf_list.append(node)
            # print(node,
            #       end = " ")
            return leaf_list
     
        # If left child exists,
        # check for leaf recursively
        if node.left:
            self.outputLeafNodes(node.left, leaf_list)
     
        # If right child exists,
        # check for leaf recursively
        if node.right:
            self.outputLeafNodes(node.right, leaf_list)
        return leaf_list
    @classmethod
    def findParent(self,node : Node):
        print(node)
        if node == self.root:
            return

        # If current node is
        # the required node
        if (node.left == node or node.right == node):
        
            # Print its parent
            print('n',node)
            return node
        else:
        
            # Recursive calls
            # for the children
            # of the current node
            # Current node is now
            # the new parent
            self.findParent(node.left)
            self.findParent(node.right)
        return node




# spec = '!((2&(1|3))&!(2|4))'
# spec = '(1&(2|4))'

def main():

    def find_parens(s):
        toret = {}
        pstack = []

        for i, c in enumerate(s):
            if c == '(':
                pstack.append(i)
            elif c == ')':
                if len(pstack) == 0:
                    raise IndexError("No matching closing parens at: " + str(i))
                toret[pstack.pop()] = i

        if len(pstack) > 0:
            raise IndexError("No matching opening parens at: " + str(pstack.pop()))

        return toret

    # def expand(spec,ap):
    #     ''' !(a&b) -> (!a|!b)
    #         !(a|b) -> (!a&!b)
    #     '''
    #     # neg_idx = spec.find('!(')
    #     # print(neg_idx)

    #     # paren_dict = find_parens(spec)
    #     # print(paren_dict)
    #     # open_paren = neg_idx + 1
    #     # close_paren = paren_dict[open_paren]+1
    #     # new_spec = spec[neg_idx:close_paren]
    #     # print(spec[neg_idx:close_paren])
    #     # new_spec = '!(1&(2|4))'
    #     tree=Tree.build(spec)
    #     # op = tree.root.symbol

    #     # if op == '&':
    #     #     tree.root.symbol = '&'
        
    #     # print('t',tree)
    #     # tree.expand_neg()
    #     print('a',tree)
    #     gr = tree.evaluate(ap)
    #     print('eval',gr)
        
    # spec = '(1&2)'
    # expand(spec,'a')

if __name__ == '__main__':
    main()



''' SYNTHESIS
check if buchi intersection is valid:
    if all ap_0 values conflict with other ap bindings: remove node (for each binding, replace ap_0 with ap_binding and check)
    BUT how to construct buchi intersection if current spec looks like the binding stuff too? how to update sync?
        for the current buchi and its current bindings, ignore bindings that it doesn't do

A x Buchi. For each formula:
    assign bindings_A = {all possible bindings}
    for each binding not associated with APs that exist in A:
        if not every AP is False:
            remove binding from bindings_A
    for each binding associated with APs that exist in A:
        find conflicts in formula with A (ignore parts of the formula with different bindings)
        if conflict:
            remove binding from bindings_A

    if bindings_A is empty:
        don't add node
    # find all bindings associated with APs that exist in A. If none, don't add node.

    
    don't add node if:
        if all bindings have conflicts
        # the bindings that have conflicts are part of the set of bindings in previous node (don't think this is necessary)

afterwards, need to find if there's a path with at least one consistent binding for each transition

each node has:
    required bindings
    bindings that conflict
    bindings that don't matter (another robot would need to do)
if every binding of node that conflicts previous node (n1 && n2's bindings = empty set): don't add node
if node has one binding that conflicts: either remove that binding from previous 

the problem: there needs to be a node for each combination of valid bindings.
Ex: a node can do all three bindings. the next node can only do one of those bindings

---
another method for A x Buchi. For each formula:
    For each binding, check if APs of A conflict with truth values of Buchi formula. store set of bindings that are valid

when synthesizing path thru A x Buchi:
    would i need to check every combination fo possible bindings?


synthesize shortest path through 

HOW TO DETERMINE COST?

synchronization:
idle in closest self-looping node?
what happens if the paths each robot takes through buchi is different and therefore requires different aps to be true/false

'''