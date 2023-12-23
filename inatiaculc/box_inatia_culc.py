# Boxの計算
def calculate_box_inertia_with_offset(m, w, d, h, dx, dy, dz):
    # 基本的な慣性モーメント
    Iw = (m / 12.0) * (d**2 + h**2)
    Id = (m / 12.0) * (w**2 + h**2)
    Ih = (m / 12.0) * (w**2 + d**2)

    # 平行軸定理による修正
    Iw += m * (dy**2 + dz**2)
    Id += m * (dx**2 + dz**2)
    Ih += m * (dx**2 + dy**2)

    # 慣性積の計算
    ixy = -m * dx * dy
    ixz = -m * dx * dz
    iyz = -m * dy * dz

    # 非常に小さい値を0.0として扱う
    ixy = 0.0 if abs(ixy) < 1e-10 else ixy
    ixz = 0.0 if abs(ixz) < 1e-10 else ixz
    iyz = 0.0 if abs(iyz) < 1e-10 else iyz

    print(f'BOX ixx="{Iw:.5f}" iyy="{Id:.5f}" izz="{Ih:.5f}" ixy="{ixy:.5f}" ixz="{ixz:.5f}" iyz="{iyz:.5f}"')

# 数値入力を取得する関数
def get_float_input(prompt):
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("無効な入力です. 数字を入力してください.")

# Inertia計算機
selection = "START"
while selection.upper() != "Q":
    print("====================")
    print("Boxの物体についてURDF用の慣性テンソルを計算します. :")
    print("[1] 入力単位 (kg, m) width(x-axis)*depth(y-axis)*height(z-axis)")
    print("[2] 入力単位 (g, mm) width(x-axis)*depth(y-axis)*height(z-axis)")
    print("[Q] 終了")

    selection = input(">>")

    if selection == "1":
        mass = get_float_input("mass>>")
        width = get_float_input("x-axis length (Okuyuki) >>")
        depth = get_float_input("y-axis length (YokoHaba)>>")
        height = get_float_input("z-axis length (Takasa) >>")
        dx = get_float_input("x-axis offset >>")
        dy = get_float_input("y-axis offset >>")
        dz = get_float_input("z-axis offset >>")
        calculate_box_inertia_with_offset(m=mass, w=width, d=depth, h=height, dx=dx, dy=dy, dz=dz)

    if selection == "2":
        mass = get_float_input("mass>>")*0.001
        width = get_float_input("x-axis length (Okuyuki) >>")*0.001
        depth = get_float_input("y-axis length (YokoHaba)>>")*0.001
        height = get_float_input("z-axis length (Takasa) >>")*0.001
        dx = get_float_input("x-axis offset >>")*0.001
        dy = get_float_input("y-axis offset >>")*0.001
        dz = get_float_input("z-axis offset >>")*0.001
        calculate_box_inertia_with_offset(m=mass, w=width, d=depth, h=height, dx=dx, dy=dy, dz=dz)
