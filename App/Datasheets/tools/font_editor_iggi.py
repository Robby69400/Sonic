import tkinter as tk
from tkinter import ttk, messagebox
import re
# --- Your fonts (Insert full arrays here) ---
gFontBig = [[0x00]*14]
gFontBig_chars = "!" + "\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~"
gFontBigDigits = [[0x00]*20]
gFontBigDigits_chars = "0123456789+-. "
gFontSmall = [[0x00]*6]
gFontSmall_chars = " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~"
gFont3x5 = [[0x00]*3]
gFont3x5_chars = " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~"

class FontTab:
    def __init__(self, name, width, height, chars, parent):
        self.name, self.width, self.height, self.chars = name, width, height, chars
        self.frame = ttk.Frame(parent.notebook)
        self.canvas = tk.Canvas(self.frame, width=width*30, height=height*30, bg="white")
        self.canvas.pack(pady=10)
        self.grid = [[0]*width for _ in range(height)]
        self.rects = [[None]*width for _ in range(height)]
        for y in range(height):
            for x in range(width):
                self.rects[y][x] = self.canvas.create_rectangle(x*30, y*30, (x+1)*30, (y+1)*30, fill="white", outline="gray")
        self.canvas.bind("<Button-1>", self.toggle_pixel)
        self.canvas.bind("<B1-Motion>", self.toggle_pixel)
        
        controls = ttk.Frame(self.frame)
        controls.pack(fill=tk.X, padx=10)
        ttk.Label(controls, text="Char:").pack(side=tk.LEFT)
        self.char_var = tk.StringVar()
        self.char_combo = ttk.Combobox(controls, textvariable=self.char_var, values=list(chars), width=5)
        self.char_combo.pack(side=tk.LEFT, padx=5)
        self.char_combo.bind("<<ComboboxSelected>>", self.load_char)
        ttk.Button(controls, text="Get Hex", command=self.generate_hex).pack(side=tk.LEFT, padx=5)
        ttk.Button(controls, text="Clear", command=self.clear_grid).pack(side=tk.LEFT, padx=5)

    def toggle_pixel(self, event):
        x, y = event.x // 30, event.y // 30
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = 1
            self.canvas.itemconfig(self.rects[y][x], fill="black")

    def clear_grid(self):
        self.grid = [[0]*self.width for _ in range(self.height)]
        for y in range(self.height):
            for x in range(self.width):
                self.canvas.itemconfig(self.rects[y][x], fill="white")

    def load_char(self, event=None):
        self.clear_grid()
        c = self.char_var.get()
        if not c: return
        try:
            idx = self.chars.find(c)
            font_data = globals()[self.name][idx]
            for col in range(self.width):
                val = font_data[col]
                for row in range(min(self.height, 8)):
                    if val & (1 << row): self.set_pix(col, row)
                if self.height > 8 and col + self.width < len(font_data):
                    val2 = font_data[col + self.width]
                    for row in range(8): self.set_pix(col, row + 8)
        except: pass

    def set_pix(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = 1
            self.canvas.itemconfig(self.rects[y][x], fill="black")

    def generate_hex(self):
        res = []
        for col in range(self.width):
            val = 0
            for row in range(min(self.height, 8)):
                if self.grid[row][col]: val |= (1 << row)
            res.append(f"0x{val:02X}")
        if self.height > 8:
            for col in range(self.width):
                val = 0
                for row in range(8, self.height):
                    if self.grid[row][col]: val |= (1 << (row-8))
                res.append(f"0x{val:02X}")
        print(f"// '{self.char_var.get()}'\n{" ,".join(res)}")

class FontEditor(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Font Editor Iggi - Translated")
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(expand=True, fill="both", padx=10, pady=10)
        self.tabs = {
            "gFontBig": FontTab("gFontBig", 7, 16, gFontBig_chars, self),
            "gFontBigDigits": FontTab("gFontBigDigits", 10, 16, gFontBigDigits_chars, self),
            "gFontSmall": FontTab("gFontSmall", 6, 8, gFontSmall_chars, self),
            "gFont3x5": FontTab("gFont3x5", 3, 5, gFont3x5_chars, self)
        }
        for t in self.tabs.values(): self.notebook.add(t.frame, text=f"{t.name} ({t.width}x{t.height})")
        
        input_frame = ttk.Frame(self)
        input_frame.pack(fill=tk.BOTH, padx=10, pady=5)
        self.input_text = tk.Text(input_frame, height=5)
        self.input_text.pack(side=tk.LEFT, expand=True, fill=tk.BOTH)
        ttk.Button(input_frame, text="Parse Input", command=self.parse_input).pack(side=tk.RIGHT, padx=5)
        self.notification_label = ttk.Label(self, text="", foreground="green")
        self.notification_label.pack()

    def parse_input(self):
        raw = self.input_text.get("1.0", tk.END).strip()
        hex_m = re.findall(r'0x[0-9A-Fa-f]{2}', raw)
        if not hex_m: return
        vals = [int(h, 16) for h in hex_m]
        ln = len(vals)
        target = next((t for t in self.tabs.values() if (t.height <= 8 and ln == t.width) or (t.height > 8 and ln == t.width*2)), None)
        if target:
            self.notebook.select(target.frame)
            target.clear_grid()
            for col in range(target.width):
                v = vals[col]
                for r in range(min(target.height, 8)):
                    if v & (1 << r): target.set_pix(col, r)
                if target.height > 8:
                    v2 = vals[col + target.width]
                    for r in range(8): target.set_pix(col, r + 8)
            self.notification_label.config(text="Successfully parsed!")
            self.after(2000, lambda: self.notification_label.config(text=""))
        else: messagebox.showerror("Error", f"Could not find matching font for {ln} bytes")

if __name__ == "__main__": FontEditor().mainloop()