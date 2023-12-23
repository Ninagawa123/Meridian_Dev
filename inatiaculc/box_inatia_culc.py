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

    print(f'BOX ixx="{Iw:.5f}" iyy="{Id:.5f}" izz="{Ih:.5f}" ixy="0.0" ixz="0.0" iyz="0.0"')

# Inertia計算機
selection = "START"
while selection.upper() != "Q":
    print("====================")
    print("Boxの物体についてURDF用の慣性テンソルを計算します.:")
    print("[1] 計算 width(x-axis)*depth(y-axis)*height(z-axis)")
    print("[Q] 終了")

    selection = input(">>")

    if selection == "1":
        mass = float(input("mass>>"))
        width =  float(input("x-axis length (Okuyuki) >>"))
        depth =  float(input("y-axis length (YokoHaba)>>"))
        height = float(input("z-axis length (Takasa)  >>"))
        dx = float(input("x-axis offset>>"))
        dy = float(input("y-axis offset>>"))
        dz = float(input("z-axis offset>>"))
        calculate_box_inertia_with_offset(m=mass, w=width, d=depth, h=height, dx=dx, dy=dy, dz=dz)
