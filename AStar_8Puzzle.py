# 初始状态
init_state = [
    [1, 8, 7],
    [3, 0, 5],
    [4, 6, 2]
]
# 目标状态
goal_state = [
    [1, 2, 3],
    [4, 5, 6],
    [7, 8, 0]
]
# 目标状态 值-位置表
goal_dic = {
    1:(0,0), 2:(0,1), 3:(0,2),
    4:(1,0), 5:(1,1), 6:(1,2),
    7:(2,0), 8:(2,1), 0:(2,2)
}
# 输出状态
def PrintState(state):
    for i in state: print(i)
# 复制状态
def CopyState(state):
    s = []
    for i in state: s.append(i[:])
    return s
# 获取空格的位置
def GetSpace(state):
    for y in range(len(state)):
        for x in range(len(state[y])):
            if state[y][x] == 0: return y, x
# 获取空格上移后的状态，不改变原状态
def MoveUp(state):
    s = CopyState(state)
    y, x = GetSpace(s)
    s[y][x], s[y - 1][x] = s[y - 1][x], s[y][x]
    return s
# 获取空格下移后的状态，不改变原状态
def MoveDown(state):
    s = CopyState(state)
    y, x = GetSpace(s)
    s[y][x], s[y + 1][x] = s[y + 1][x], s[y][x]
    return s
# 获取空格左移后的状态，不改变原状态
def MoveLeft(state):
    s = CopyState(state)
    y, x = GetSpace(s)
    s[y][x], s[y][x - 1] = s[y][x - 1], s[y][x]
    return s
# 获取空格右移后的状态，不改变原状态
def MoveRight(state):
    s = CopyState(state)
    y, x = GetSpace(s)
    s[y][x], s[y][x + 1] = s[y][x + 1], s[y][x]
    return s
# 获取两个状态之间的启发距离
def GetDistance(src, dest):
    dic, d = goal_dic, 0
    for i in range(len(src)):
        for j in range(len(src[i])):
            pos = dic[src[i][j]]
            y, x= pos[0], pos[1]
            d += abs(y - i) + abs(x - j)
    return d
# 获取指定状态下的操作
def GetActions(state):
    acts = []
    y, x = GetSpace(state)
    if x > 0:acts.append(MoveLeft)
    if y > 0:acts.append(MoveUp)
    if x < len(state[0]) - 1:acts.append(MoveRight)
    if y < len(state[0]) - 1: acts.append(MoveDown)
    return acts
# 用于统一操作序列的函数
def Start(state):
    return
# 边缘队列中的节点类
class Node:
    state = None   # 状态
    value = -1     # 启发值
    step = 0       # 初始状态到当前状态的距离（步数）
    action = Start  # 到达此节点所进行的操作
    parent = None,  # 父节点
    # 用状态和步数构造节点对象
    def __init__(self, state, step, action, parent):
        self.state = state
        self.step = step
        self.action = action
        self.parent = parent
        # 计算估计距离
        self.value = GetDistance(state, goal_state) + step
# 获取拥有最小启发值的元素索引
def GetMinIndex(queue):
    index = 0
    for i in range(len(queue)):
        node = queue[i]
        if node.value < queue[index].value:
            index = i
    return index
# 将状态转换为整数
def toInt(state):
    value = 0
    for i in state:
        for j in i:
            value = value * 10 + j
    return value
# A*算法寻找初始状态到目标状态的路径
def AStar(init, goal):
    # 边缘队列初始已有源状态节点
    queue = [Node(init, 0, Start, None)]
    visit = {}  # 访问过的状态表
    count = 0   # 循环次数
    # 队列没有元素则查找失败
    while queue:
        # 获取拥有最小估计距离的节点索引
        index = GetMinIndex(queue)
        node = queue[index]
        visit[toInt(node.state)] = True
        count += 1
        if node.state == goal:
            return node, count
        del queue[index]
        # 扩展当前节点
        for act in GetActions(node.state):
            # 获取此操作下到达的状态节点并将其加入边缘队列中
            near = Node(act(node.state), node.step + 1, act, node)
            if toInt(near.state) not in visit:
                queue.append(near)
    return None, count

# 将链表倒序，返回链头和链尾
def reverse(node):
    if node.parent == None:
        return node, node
    head, rear = reverse(node.parent)
    rear.parent, node.parent = node, None
    return head, node

node, count = AStar(init_state, goal_state)
if node == None:
    print("无法从初始状态到达目标状态！")
else:
    print("搜索成功，循环次数：", count)
    node, rear = reverse(node)
    count = 0
    while node:
        # 启发值包括从起点到此节点的距离
        print("第", count + 1, "步：", node.action.__name__, "启发值为：", count, "+", node.value - count)
        PrintState(node.state)
        node = node.parent
        count += 1