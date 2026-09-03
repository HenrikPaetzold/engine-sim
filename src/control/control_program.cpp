#include "../../include/control/control_program.h"

control::ControlProgram::ControlProgram() {
    m_compiled = false;
}

control::ControlProgram::~ControlProgram() {
    destroy();
}

void control::ControlProgram::destroy() {
    for (ControlBlock *block : m_blocks) {
        delete block;
    }

    m_blocks.clear();
    m_order.clear();
    m_values.clear();
    m_compiled = false;
}

int control::ControlProgram::addBlock(ControlBlock *block) {
    if (block == nullptr) return -1;

    block->m_index = static_cast<int>(m_blocks.size());
    m_blocks.push_back(block);
    m_compiled = false;

    return block->m_index;
}

control::ControlBlock *control::ControlProgram::getBlock(int index) const {
    if (index < 0 || index >= static_cast<int>(m_blocks.size())) return nullptr;
    return m_blocks[index];
}

control::ControlBlock *control::ControlProgram::findBlock(const std::string &name) const {
    for (ControlBlock *block : m_blocks) {
        if (block->m_name == name) return block;
    }

    return nullptr;
}

bool control::ControlProgram::sort() {
    const int count = static_cast<int>(m_blocks.size());

    std::vector<int> mark(count, 0);
    std::vector<int> stack;

    m_order.clear();
    m_order.reserve(count);

    for (int start = 0; start < count; ++start) {
        if (mark[start] == 2) continue;

        stack.clear();
        stack.push_back(start);

        while (!stack.empty()) {
            const int current = stack.back();

            if (mark[current] == 0) {
                mark[current] = 1;

                ControlBlock *block = m_blocks[current];
                if (!block->breaksCycles()) {
                    for (int i = 0; i < block->getOperandCount(); ++i) {
                        const int operand = block->getOperand(i);
                        if (operand < 0 || operand >= count) continue;

                        if (mark[operand] == 1) {
                            m_error =
                                "cycle through '" + block->m_name
                                + "' and '" + m_blocks[operand]->m_name
                                + "'; insert a delay block to break it";
                            return false;
                        }

                        if (mark[operand] == 0) stack.push_back(operand);
                    }
                }
            }
            else {
                stack.pop_back();

                if (mark[current] == 1) {
                    mark[current] = 2;
                    m_order.push_back(current);
                }
            }
        }
    }

    return true;
}

bool control::ControlProgram::compile() {
    m_error.clear();
    m_compiled = false;

    if (!sort()) return false;

    m_values.assign(m_blocks.size(), 0.0);
    m_compiled = true;

    reset();

    return true;
}

void control::ControlProgram::reset() {
    for (ControlBlock *block : m_blocks) {
        block->reset();
    }

    for (size_t i = 0; i < m_values.size(); ++i) {
        m_values[i] = 0.0;
    }

    m_outputs.clearValues();
}

void control::ControlProgram::update(double dt) {
    if (!m_compiled) return;

    BlockContext context;
    context.values = m_values.data();
    context.inputs = &m_inputs;
    context.outputs = &m_outputs;
    context.dt = dt;

    for (int index : m_order) {
        ControlBlock *block = m_blocks[index];
        block->m_output = block->evaluate(context);
        m_values[index] = block->m_output;
    }

    for (int index : m_order) {
        m_blocks[index]->latch(context);
    }
}

double control::ControlProgram::getValue(int index) const {
    if (index < 0 || index >= static_cast<int>(m_values.size())) return 0.0;
    return m_values[index];
}
