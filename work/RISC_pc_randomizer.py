# import random
# import os
# os.chdir('work')

# instruction_set = [
#     'load', 'store', 'loadi', 'storei', 'mov', 'or', 'inv', 'and', 'add', 'sub', 
#     'addc', 'subc', 'jump', 'bz', 'bnz', 'bc', 'bv', 'jal', 'jral', 'ret'
# ]

# non_flow_control_instructions = [
#     'load', 'store', 'loadi', 'storei', 'mov', 'or', 'inv', 'and', 'add', 'sub', 
#     'addc', 'subc'
# ]

# flow_control_instructions = [
#     'jump', 'bz', 'bnz', 'bc', 'bv', 'jal', 'jral', 'ret'
# ]

# def generate_random_register():
#     return random.randint(0, 31)

# def generate_random_immediate(bits):
#     return random.randint(0, (1 << bits) - 1)

# def generate_random_instruction(instr_set):
#     instr = random.choice(instr_set)
#     if instr in ['load','jral']:
#         return f"{instr} {generate_random_register()},{generate_random_register()},{generate_random_immediate(12)}"
#     elif instr in ['store', 'bz', 'bnz', 'bc', 'bv']:
#         return f"{instr} 0,{generate_random_register()},{generate_random_register()},{generate_random_immediate(12)}"
#     elif instr in ['loadi','jal']:
#         return f"{instr} {generate_random_register()},{generate_random_immediate(12)}"
#     elif instr == 'storei':
#         return f"{instr} {generate_random_register()},{generate_random_immediate(12)}"
#     elif instr in ['or', 'and', 'add', 'sub', 'addc', 'subc']:
#         return f"{instr} {generate_random_register()},{generate_random_register()},{generate_random_register()}"
#     elif instr in ['mov', 'inv']:
#         return f"{instr} {generate_random_register()},{generate_random_register()}"
#     elif instr in ['jump', 'ret']:
#         return f"{instr} {generate_random_register()}"
#     else:
#         raise ValueError(f"Unrecognized instruction: {instr}")

# def generate_random_assembly_code(num_instructions, no_flow_control_count):
#     instructions = []
    
#     # Generate first set without flow control instructions
#     for _ in range(no_flow_control_count):
#         instructions.append(generate_random_instruction(non_flow_control_instructions))
    
#     # Generate remaining instructions with all types
#     for _ in range(num_instructions - no_flow_control_count):
#         instructions.append(generate_random_instruction(instruction_set))
    
#     return instructions

# def write_assembly_code_to_file(filename, assembly_code):
#     with open(filename, 'w') as file:
#         for instruction in assembly_code:
#             file.write(instruction + '\n')

# num_instructions = 50  # Total number of instructions
# no_flow_control_count = 150  # Number of instructions without flow control

# random_assembly_code = generate_random_assembly_code(num_instructions, no_flow_control_count)

# # Save the generated assembly code to a file
# output_file = 'random_program.asm'
# write_assembly_code_to_file(output_file, random_assembly_code)

# print(f"Generated {num_instructions} random instructions and saved to {output_file}")

import random
import os
os.chdir('work')

instruction_set = [
    'load', 'store', 'loadi', 'storei', 'mov', 'or', 'inv', 'and', 'add', 'sub', 
    'addc', 'subc', 'jump', 'bz', 'bnz', 'bc', 'bv', 'jal', 'jral', 'ret'
]

non_flow_control_instructions = [
    'load', 'store', 'loadi', 'storei', 'mov', 'or', 'inv', 'and', 'add', 'sub', 
    'addc', 'subc'
]

flow_control_instructions = [
    'jump', 'bz', 'bnz', 'bc', 'bv', 'jal', 'jral', 'ret'
]

def generate_random_register():
    return random.randint(0, 31)

def generate_random_immediate(bits):
    return random.randint(0, (1 << bits) - 1)

def generate_random_instruction(instr_set):
    instr = random.choice(instr_set)
    if instr in ['load','jral']:
        return f"{instr} {generate_random_register()},{generate_random_register()},{generate_random_immediate(12)}"
    elif instr in ['store', 'bz', 'bnz', 'bc', 'bv']:
        return f"{instr} 0,{generate_random_register()},{generate_random_register()},{generate_random_immediate(12)}"
    elif instr in ['loadi','jal']:
        return f"{instr} {generate_random_register()},{generate_random_immediate(12)}"
    elif instr == 'storei':
        return f"{instr} {generate_random_register()},{generate_random_immediate(12)}"
    elif instr in ['or', 'and', 'add', 'sub', 'addc', 'subc']:
        return f"{instr} {generate_random_register()},{generate_random_register()},{generate_random_register()}"
    elif instr in ['mov', 'inv']:
        return f"{instr} {generate_random_register()},{generate_random_register()}"
    elif instr in ['jump', 'ret']:
        return f"{instr} {generate_random_register()}"
    else:
        raise ValueError(f"Unrecognized instruction: {instr}")

def generate_random_assembly_code(total_instructions, num_flow_ctrl_instructions):
    num_non_flow_ctrl_instructions = total_instructions - num_flow_ctrl_instructions
    
    non_flow_ctrl_instructions = [
        generate_random_instruction(non_flow_control_instructions) 
        for _ in range(num_non_flow_ctrl_instructions)
    ]
    
    flow_ctrl_instructions = [
        generate_random_instruction(flow_control_instructions) 
        for _ in range(num_flow_ctrl_instructions)
    ]
    
    # Mix the instructions
    mixed_instructions = non_flow_ctrl_instructions + flow_ctrl_instructions
    random.shuffle(mixed_instructions)
    
    return mixed_instructions

def write_assembly_code_to_file(filename, assembly_code):
    with open(filename, 'w') as file:
        for instruction in assembly_code:
            file.write(instruction + '\n')

total_instructions = 180  # Total number of instructions
num_flow_ctrl_instructions = 20  # Number of flow control instructions

random_assembly_code = generate_random_assembly_code(total_instructions, num_flow_ctrl_instructions)

# Save the generated assembly code to a file
output_file = 'random_program.asm'
write_assembly_code_to_file(output_file, random_assembly_code)

print(f"Generated {total_instructions} random instructions (including {num_flow_ctrl_instructions} flow control instructions) and saved to {output_file}")
