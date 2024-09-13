import requests
import os

def download_pdf(url, save_path):
    try:
        # Send a GET request to the URL
        response = requests.get(url)
        response.raise_for_status()  # Check if the request was successful

        # Ensure the save directory exists
        os.makedirs(os.path.dirname(save_path), exist_ok=True)

        # Write the content to a file in binary mode
        with open(save_path, 'wb') as pdf_file:
            pdf_file.write(response.content)
        print(f"PDF downloaded and saved to {save_path}")
        return True
    except requests.exceptions.RequestException as e:
        print(f"Error downloading the PDF: {e}")
        return False


with open("ctm.kicad_sym", "r") as f:
    lines = f.readlines()

windowsProblematicChars = "?(){}[]\\\""

remainingIssues = 0
problematicDatasheetsSolved = 0
for i, line in enumerate(lines):
    if '"Datasheet"' in line and not "KICAD_CTM" in line and not '""' in line and not '"~"' in line:
        if "KICAD_ctm" in line:
            lines[i] = "KICAD_CTM".join(line.split("KICAD_ctm"))
            continue
        linedata = line.split(" ")
        datasheet=linedata[-1][1:-2]
        datasheet_downloaded = datasheet.split("/")[-1]
        if datasheet_downloaded[-4:] != ".pdf" or any([c in datasheet_downloaded for c in windowsProblematicChars]):
            if datasheet_downloaded[-4:] != ".pdf" and ".pdf" in datasheet_downloaded:
                datasheet_downloaded = datasheet_downloaded.split(".pdf")[0] + ".pdf"
            if any([c in datasheet_downloaded for c in windowsProblematicChars]):
                datasheet_downloaded = datasheet_downloaded[:10] + ".pdf"
        linedata[-1] = '"${KICAD_CTM}/datasheets/' + datasheet_downloaded + '"\n'
        reassembled_line = " ".join(linedata)
        if "www.st.com" in datasheet or "www.mouser." in datasheet:
            print(f"\tmanual intervention required for {datasheet}\n\n")
            remainingIssues += 1
            continue
        print(f"attempting to download {datasheet}")
        if download_pdf(datasheet, f"./datasheets/{datasheet_downloaded}"):
            lines[i] = reassembled_line
            problematicDatasheetsSolved += 1
        else: 
            remainingIssues += 1
            print(f"check download link: {datasheet}")

print (f"Solved {problematicDatasheetsSolved} datasheets, {remainingIssues} could not be resolved automatically")

with open("ctm.kicad_sym", "w") as f:
    f.writelines(lines)