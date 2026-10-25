#!/usr/bin/env python3
"""
Font Editor Iggi — Éditeur de fontes bitmap 10×16 pixels (gFontBigDigits)

Permet de dessiner des caractères dans une grille, générer le code C,
et exporter/importer des données hexadécimales.

Usage: python font_editor_11x20.py
"""

import tkinter as tk
from tkinter import ttk, messagebox
import re


# --- Données de police par défaut (à remplir avec vos glyphes C) ---
# gFontBigDigits : 11 glyphes (0-9 + "-"), 20 octets chacun (10×16 pixels)
gFontBigDigits = [[0x00] * 20 for _ in range(11)]
gFontBigDigits_chars = "0123456789-"

gFontBig = [[0x00] * 14]
gFontBig_chars = "!\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~"

gFontSmall = [[0x00] * 6]
gFontSmall_chars = " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~"

gFont3x5 = [[0x00] * 3]
gFont3x5_chars = " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~"


class FontTab:
    """Onglet d'édition pour une police bitmap."""

    def __init__(self, name, width, height, chars, parent, bytes_per_glyph=None):
        self.name = name
        self.width = width
        self.height = height
        self.chars = chars
        self.bytes_per_glyph = bytes_per_glyph or width * (2 if height > 8 else 1)

        self.frame = ttk.Frame(parent.notebook)
        
        self.cell_size = 24
        self.canvas = tk.Canvas(
            self.frame,
            width=width * self.cell_size,
            height=height * self.cell_size,
            bg="white",
            highlightthickness=1,
            highlightbackground="#888"
        )
        self.canvas.pack(pady=10)

        # Grille de pixels et rectangles Canvas
        self.grid = [[0] * width for _ in range(height)]
        self.rects = [[None] * width for _ in range(height)]
        
        for y in range(height):
            for x in range(width):
                self.rects[y][x] = self.canvas.create_rectangle(
                    x * self.cell_size, y * self.cell_size,
                    (x + 1) * self.cell_size, (y + 1) * self.cell_size,
                    fill="white", outline="#b8b8b8"
                )

        # Liaisons souris
        self.canvas.bind("<Button-1>", self.paint_pixel)
        self.canvas.bind("<B1-Motion>", self.paint_pixel)
        self.canvas.bind("<Button-3>", self.erase_pixel)
        self.canvas.bind("<B3-Motion>", self.erase_pixel)

        # Contrôles
        controls = ttk.Frame(self.frame)
        controls.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(controls, text="Caractère :").pack(side=tk.LEFT)
        self.char_var = tk.StringVar(value=chars[0] if chars else "")
        self.char_combo = ttk.Combobox(
            controls, textvariable=self.char_var,
            values=list(chars), width=5, state="readonly"
        )
        self.char_combo.pack(side=tk.LEFT, padx=5)
        # On intercepte le changement via un bind, mais on garde la trace de l'ancien caractère pour sauvegarder
        self.previous_char = self.char_var.get()
        self.char_combo.bind("<<ComboboxSelected>>", self.on_char_change)

        ttk.Button(controls, text="Inverser (Négatif)", command=self.invert_grid).pack(side=tk.LEFT, padx=5)
        ttk.Button(controls, text="Générer le C", command=self.generate_hex).pack(side=tk.LEFT, padx=5)
        ttk.Button(controls, text="Effacer", command=self.clear_grid).pack(side=tk.LEFT, padx=5)
        ttk.Button(controls, text="Tout exporter", command=self.export_all).pack(side=tk.LEFT, padx=5)

        # Zone de sortie
        self.output = tk.Text(self.frame, height=5, wrap=tk.NONE, font=("Courier", 9))
        self.output.pack(fill=tk.X, padx=10, pady=5)

        # Chargement initial
        self.load_char()

    def save_current_char(self):
        """Sauvegarder la grille actuelle dans le tableau de données du caractère précédent."""
        char = self.previous_char
        if not char:
            return
        try:
            idx = self.chars.index(char)
            globals()[self.name][idx] = self.grid_to_bytes()
        except (ValueError, IndexError, KeyError):
            pass

    def on_char_change(self, event=None):
        """Gérer le changement de caractère en sauvegardant l'ancien et en chargeant le nouveau."""
        self.save_current_char()
        self.previous_char = self.char_var.get()
        self.load_char()

    def paint_pixel(self, event):
        """Dessiner un pixel avec le bouton gauche."""
        x, y = event.x // self.cell_size, event.y // self.cell_size
        if 0 <= x < self.width and 0 <= y < self.height:
            self.set_pixel(x, y, 1)

    def erase_pixel(self, event):
        """Effacer un pixel avec le bouton droit."""
        x, y = event.x // self.cell_size, event.y // self.cell_size
        if 0 <= x < self.width and 0 <= y < self.height:
            self.set_pixel(x, y, 0)

    def set_pixel(self, x, y, value):
        """Définir l'état d'un pixel."""
        self.grid[y][x] = value
        self.canvas.itemconfig(self.rects[y][x], fill="black" if value else "white")

    def clear_grid(self):
        """Effacer toute la grille."""
        for y in range(self.height):
            for x in range(self.width):
                self.set_pixel(x, y, 0)

    def invert_grid(self):
        """Inverser tous les pixels de la grille (négatif)."""
        for y in range(self.height):
            for x in range(self.width):
                new_val = 0 if self.grid[y][x] else 1
                self.set_pixel(x, y, new_val)

    def load_char(self, event=None):
        """Charger un caractère depuis les données de police."""
        self.clear_grid()
        char = self.char_var.get()
        if not char:
            return
        try:
            idx = self.chars.index(char)
            font_data = globals()[self.name][idx]
        except (ValueError, IndexError, KeyError):
            return

        # Débit des octets pour fonte bipage (hauteur > 8)
        if self.height > 8 and len(font_data) >= self.width * 2:
            for x in range(self.width):
                top_byte = font_data[x]
                bottom_byte = font_data[x + self.width]
                for bit in range(8):
                    if top_byte & (1 << bit):
                        self.set_pixel(x, bit, 1)
                    if bottom_byte & (1 << bit):
                        self.set_pixel(x, bit + 8, 1)
        elif len(font_data) >= self.width:
            for x in range(self.width):
                value = font_data[x]
                for bit in range(min(self.height, 8)):
                    if value & (1 << bit):
                        self.set_pixel(x, bit, 1)

    def grid_to_bytes(self):
        """Convertir la grille en tableau d'octets C."""
        result = []
        # Page haute (lignes 0-7)
        for x in range(self.width):
            value = 0
            for y in range(min(8, self.height)):
                if self.grid[y][x]:
                    value |= (1 << y)
            result.append(value)

        # Page basse (lignes 8-15) si applicable
        if self.height > 8:
            for x in range(self.width):
                value = 0
                for y in range(8, self.height):
                    if self.grid[y][x]:
                        value |= (1 << (y - 8))
                result.append(value)
        return result

    def generate_hex(self):
        """Générer et copier le code C pour le glyphe courant."""
        # On met à jour les données du glyphe courant avant de générer
        self.save_current_char()
        values = self.grid_to_bytes()
        char = self.char_var.get()
        
        # Formatage en lignes de 10 octets
        lines = [f"    // '{char}'"]
        lines.append("    {")
        for start in range(0, len(values), 10):
            chunk = values[start:start + 10]
            lines.append("        " + ", ".join(f"0x{v:02X}" for v in chunk) + ",")
        lines.append("    },")
        
        text = "\n".join(lines)
        self.output.delete("1.0", tk.END)
        self.output.insert("1.0", text)
        
        # Copie dans le presse-papiers
        self.clipboard_clear()
        self.clipboard_append(text)

    def export_all(self):
        """Exporter toute la table de police."""
        self.save_current_char()
        rows = []
        for char in self.chars:
            idx = self.chars.index(char)
            if idx < len(globals()[self.name]):
                data = globals()[self.name][idx]
            else:
                data = [0x00] * self.bytes_per_glyph
            rows.append(f"    // '{char}'")
            rows.append("    {" + ", ".join(f"0x{v:02X}" for v in data) + "},")
        
        text = f"const uint8_t {self.name}[{len(self.chars)}][{self.bytes_per_glyph}] = {{\n"
        text += "\n".join(rows)
        text += "\n};"
        
        self.output.delete("1.0", tk.END)
        self.output.insert("1.0", text)
        
        self.clipboard_clear()
        self.clipboard_append(text)


class FontEditor(tk.Tk):
    """Application principale de l'éditeur de fonte."""

    def __init__(self):
        super().__init__()
        self.title("Font Editor Iggi — fonte 10 × 16")
        self.geometry("860x800")

        # En-tête
        intro = ttk.Label(
            self,
            text="gFontBigDigits : 11 glyphes × 20 octets (10 colonnes × 16 pixels)",
            font=("TkDefaultFont", 11, "bold")
        )
        intro.pack(pady=(8, 0))

        # Onglets
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(expand=True, fill="both", padx=10, pady=10)

        self.tabs = {
            "gFontBigDigits": FontTab(
                "gFontBigDigits", 10, 16, gFontBigDigits_chars,
                self, bytes_per_glyph=20
            )
        }
        for tab in self.tabs.values():
            self.notebook.add(tab.frame, text=f"{tab.name} ({len(tab.chars)} glyphes)")

        # Importation
        import_frame = ttk.LabelFrame(self, text="Importer une table C")
        import_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.input_text = tk.Text(import_frame, height=5, wrap=tk.WORD, font=("Courier", 9))
        self.input_text.pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=5, pady=5)
        
        ttk.Button(import_frame, text="Importer les octets", command=self.parse_input).pack(side=tk.RIGHT, padx=5, pady=5)

        # Statut
        self.status = ttk.Label(
            self,
            text="Clic gauche : dessiner • Clic droit : effacer • Export : presse-papiers",
            foreground="#333"
        )
        self.status.pack(pady=(0, 8))

    def parse_input(self):
        """Analyser et importer des données hex depuis la zone de texte."""
        raw = self.input_text.get("1.0", tk.END)
        hex_matches = re.findall(r"0x([0-9A-Fa-f]{1,2})", raw)
        
        if not hex_matches:
            messagebox.showerror("Import", "Aucun octet hexadécimal 0xNN trouvé.")
            return

        values = [int(v, 16) for v in hex_matches]
        target = self.tabs["gFontBigDigits"]
        expected_single = target.bytes_per_glyph  # 20
        expected_full = len(target.chars) * expected_single  # 11 × 20 = 220

        if len(values) == expected_single:
            # Glyphe unique
            char = target.char_var.get()
            idx = target.chars.index(char)
            gFontBigDigits[idx] = values
            target.load_char()
            messagebox.showinfo("Import", f"Glyphe « {char} » importé ({len(values)} octets).")
            
        elif len(values) == expected_full:
            # Fonte complète
            for idx in range(len(target.chars)):
                start = idx * expected_single
                gFontBigDigits[idx] = values[start:start + expected_single]
            target.load_char()
            messagebox.showinfo("Import", f"Fonte complète importée ({len(values)} octets).")
            
        else:
            messagebox.showerror(
                "Import",
                f"Il faut {expected_single} octets pour un glyphe ou {expected_full} pour toute la fonte.\n"
                f"Reçu : {len(values)} octets."
            )


if __name__ == "__main__":
    FontEditor().mainloop()