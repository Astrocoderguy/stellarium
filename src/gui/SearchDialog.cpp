/*
 * Stellarium
 * Copyright (C) 2008 Guillaume Chereau
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Suite 500, Boston, MA  02110-1335, USA.
*/

#include "Dialog.hpp"
#include "SearchDialog.hpp"
#include "ui_searchDialogGui.h"
#include "StelApp.hpp"
#include "StelCore.hpp"
#include "StelModuleMgr.hpp"
#include "StelMovementMgr.hpp"
#include "StelTranslator.hpp"

#include "StelObjectMgr.hpp"
#include "StelUtils.hpp"

#include <QDebug>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QSettings>
#include <QString>
#include <QStringList>
#include <QTextEdit>
#include <QLineEdit>
#include <QComboBox>

#include "SimbadSearcher.hpp"

// Start of members for class CompletionLabel
CompletionLabel::CompletionLabel(QWidget* parent) : QLabel(parent), selectedIdx(0)
{
}

CompletionLabel::~CompletionLabel()
{
}

void CompletionLabel::setValues(const QStringList& v)
{
	values=v;
	updateText();
}

void CompletionLabel::appendValues(const QStringList& v)
{
	values+=v;
	updateText();
}

void CompletionLabel::clearValues()
{
	values.clear();
	selectedIdx=0;
	updateText();
}

QString CompletionLabel::getSelected()
{
	if (values.isEmpty())
		return QString();
	return values.at(selectedIdx);
}

void CompletionLabel::selectNext()
{
	++selectedIdx;
	if (selectedIdx>=values.size())
		selectedIdx=0;
	updateText();
}

void CompletionLabel::selectPrevious()
{
	--selectedIdx;
	if (selectedIdx<0)
		selectedIdx = values.size()-1;
	updateText();
}

void CompletionLabel::selectFirst()
{
	selectedIdx=0;
	updateText();
}

void CompletionLabel::updateText()
{
	QString newText;

	// Regenerate the list with the selected item in bold
	for (int i=0;i<values.size();++i)
	{
		if (i==selectedIdx)
			newText+="<b>"+values[i]+"</b>";
		else
			newText+=values[i];
		if (i!=values.size()-1)
			newText += ", ";
	}
	setText(newText);
}

// Start of members for class SearchDialog
SearchDialog::SearchDialog() : simbadReply(NULL)
{
	ui = new Ui_searchDialogForm;
	simbadSearcher = new SimbadSearcher(this);
	objectMgr = GETSTELMODULE(StelObjectMgr);
	Q_ASSERT(objectMgr);

	flagHasSelectedText = false;

	greekLetters.insert("alpha", QString(QChar(0x03B1)));
	greekLetters.insert("beta", QString(QChar(0x03B2)));
	greekLetters.insert("gamma", QString(QChar(0x03B3)));
	greekLetters.insert("delta", QString(QChar(0x03B4)));
	greekLetters.insert("epsilon", QString(QChar(0x03B5)));
    
	greekLetters.insert("zeta", QString(QChar(0x03B6)));
	greekLetters.insert("eta", QString(QChar(0x03B7)));
	greekLetters.insert("theta", QString(QChar(0x03B8)));
	greekLetters.insert("iota", QString(QChar(0x03B9)));
	greekLetters.insert("kappa", QString(QChar(0x03BA)));
	
	greekLetters.insert("lambda", QString(QChar(0x03BB)));
	greekLetters.insert("mu", QString(QChar(0x03BC)));
	greekLetters.insert("nu", QString(QChar(0x03BD)));
	greekLetters.insert("xi", QString(QChar(0x03BE)));
	greekLetters.insert("omicron", QString(QChar(0x03BF)));
	
	greekLetters.insert("pi", QString(QChar(0x03C0)));
	greekLetters.insert("rho", QString(QChar(0x03C1)));
	greekLetters.insert("sigma", QString(QChar(0x03C3))); // second lower-case sigma shouldn't affect anything
	greekLetters.insert("tau", QString(QChar(0x03C4)));
	greekLetters.insert("upsilon", QString(QChar(0x03C5)));
	
	greekLetters.insert("phi", QString(QChar(0x03C6)));
	greekLetters.insert("chi", QString(QChar(0x03C7)));
	greekLetters.insert("psi", QString(QChar(0x03C8)));
	greekLetters.insert("omega", QString(QChar(0x03C9)));

	QSettings* conf = StelApp::getInstance().getSettings();
	Q_ASSERT(conf);
	useSimbad = conf->value("search/flag_search_online", true).toBool();	
	useMirror = conf->value("search/use_simbad_mirror_url", "http://simbad.u-strasbg.fr/").toString();
}

SearchDialog::~SearchDialog()
{
	delete ui;
	if (simbadReply)
	{
		simbadReply->deleteLater();
		simbadReply = NULL;
	}
}

void SearchDialog::retranslate()
{
	if (dialog)
	{
		QString text(ui->lineEditSearchSkyObject->text());
		ui->retranslateUi(dialog);
		ui->lineEditSearchSkyObject->setText(text);
		populateMirrorList();
	}
}

void SearchDialog::styleChanged()
{
	// Nothing for now
}

// Initialize the dialog widgets and connect the signals/slots
void SearchDialog::createDialogContent()
{
	ui->setupUi(dialog);
	connect(&StelApp::getInstance(), SIGNAL(languageChanged()), this, SLOT(retranslate()));
	connect(ui->closeStelWindow, SIGNAL(clicked()), this, SLOT(close()));
	connect(ui->lineEditSearchSkyObject, SIGNAL(textChanged(const QString&)),
		this, SLOT(onSearchTextChanged(const QString&)));
	connect(ui->pushButtonGotoSearchSkyObject, SIGNAL(clicked()), this, SLOT(gotoObject()));
	onSearchTextChanged(ui->lineEditSearchSkyObject->text());
	connect(ui->lineEditSearchSkyObject, SIGNAL(returnPressed()), this, SLOT(gotoObject()));
	connect(ui->lineEditSearchSkyObject, SIGNAL(selectionChanged()), this, SLOT(setHasSelectedFlag()));

	ui->lineEditSearchSkyObject->installEventFilter(this);
	ui->RAAngleSpinBox->setDisplayFormat(AngleSpinBox::HMSLetters);
	ui->DEAngleSpinBox->setDisplayFormat(AngleSpinBox::DMSSymbols);
	ui->DEAngleSpinBox->setPrefixType(AngleSpinBox::NormalPlus);

	connect(ui->RAAngleSpinBox, SIGNAL(valueChanged()), this, SLOT(manualPositionChanged()));
	connect(ui->DEAngleSpinBox, SIGNAL(valueChanged()), this, SLOT(manualPositionChanged()));
    
	connect(ui->alphaPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->betaPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->gammaPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->deltaPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->epsilonPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->zetaPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->etaPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->thetaPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->iotaPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->kappaPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->lambdaPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->muPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->nuPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->xiPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->omicronPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->piPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->rhoPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->sigmaPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->tauPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->upsilonPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->phiPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->chiPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->psiPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));
	connect(ui->omegaPushButton, SIGNAL(clicked(bool)), this, SLOT(greekLetterClicked()));

	connect(ui->checkBoxUseSimbad, SIGNAL(clicked(bool)),
		this, SLOT(enableSimbadSearch(bool)));
	ui->checkBoxUseSimbad->setChecked(useSimbad);

	populateMirrorList();
	int idx = ui->useSimbadMirrorComboBox->findData(useMirror, Qt::UserRole, Qt::MatchCaseSensitive);
	if (idx==-1)
	{
		// Use University of Strasbourg as default
		idx = ui->useSimbadMirrorComboBox->findData(QVariant("http://simbad.u-strasbg.fr/"), Qt::UserRole, Qt::MatchCaseSensitive);
	}
	ui->useSimbadMirrorComboBox->setCurrentIndex(idx);
	connect(ui->useSimbadMirrorComboBox, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(mirrorBoxChanged(const QString&)));
}

void SearchDialog::setHasSelectedFlag()
{
	flagHasSelectedText = true;
}

void SearchDialog::enableSimbadSearch(bool enable)
{
	useSimbad = enable;
	
	QSettings* conf = StelApp::getInstance().getSettings();
	Q_ASSERT(conf);
	conf->setValue("search/flag_search_online", useSimbad);	
}

void SearchDialog::setVisible(bool v)
{
	StelDialog::setVisible(v);

	// Set the focus directly on the line edit
	if (ui->lineEditSearchSkyObject->isVisible())
		ui->lineEditSearchSkyObject->setFocus();
}

void SearchDialog::setSimpleStyle()
{
	ui->RAAngleSpinBox->setVisible(false);
	ui->DEAngleSpinBox->setVisible(false);
	ui->simbadStatusLabel->setVisible(false);
	ui->raDecLabel->setVisible(false);
}


void SearchDialog::manualPositionChanged()
{
	ui->completionLabel->clearValues();
	StelMovementMgr* mvmgr = GETSTELMODULE(StelMovementMgr);
	Vec3d pos;
	StelUtils::spheToRect(ui->RAAngleSpinBox->valueRadians(), ui->DEAngleSpinBox->valueRadians(), pos);
	mvmgr->setFlagTracking(false);
	mvmgr->moveToJ2000(pos, 0.05);
}

void SearchDialog::onSearchTextChanged(const QString& text)
{
	// This block needs to go before the trimmedText.isEmpty() or the SIMBAD result does not
	// get properly cleared.
	if (useSimbad) {
		if (simbadReply) {
			disconnect(simbadReply,
				   SIGNAL(statusChanged()),
				   this,
				   SLOT(onSimbadStatusChanged()));
			delete simbadReply;
			simbadReply=NULL;
		}
		simbadResults.clear();
	}

	QString trimmedText = text.trimmed().toLower();
	if (trimmedText.isEmpty()) {
		ui->completionLabel->clearValues();
		ui->completionLabel->selectFirst();
		ui->simbadStatusLabel->setText("");
		ui->pushButtonGotoSearchSkyObject->setEnabled(false);
	} else {
		if (useSimbad) {
			simbadReply = simbadSearcher->lookup(useMirror, trimmedText, 3);
			onSimbadStatusChanged();
			connect(simbadReply, SIGNAL(statusChanged()), this, SLOT(onSimbadStatusChanged()));
		}

		QString greekText = substituteGreek(trimmedText);
		QStringList matches;
		if(greekText != trimmedText) {
			matches = objectMgr->listMatchingObjectsI18n(trimmedText, 3);
			matches += objectMgr->listMatchingObjectsI18n(greekText, (5 - matches.size()));
		} else {
			matches = objectMgr->listMatchingObjectsI18n(trimmedText, 5);
		}

		ui->completionLabel->setValues(matches);
		ui->completionLabel->selectFirst();

		// Update push button enabled state
		ui->pushButtonGotoSearchSkyObject->setEnabled(true);
	}
}

// Called when the current simbad query status changes
void SearchDialog::onSimbadStatusChanged()
{
	Q_ASSERT(simbadReply);
	if (simbadReply->getCurrentStatus()==SimbadLookupReply::SimbadLookupErrorOccured)
	{
		ui->simbadStatusLabel->setText(QString("Simbad Lookup Error: ")+simbadReply->getErrorString());
		if (ui->completionLabel->isEmpty())
			ui->pushButtonGotoSearchSkyObject->setEnabled(false);
	}
	else
	{
		ui->simbadStatusLabel->setText(QString("Simbad Lookup: ")+simbadReply->getCurrentStatusString());
		// Query not over, don't disable button
		ui->pushButtonGotoSearchSkyObject->setEnabled(true);
	}

	if (simbadReply->getCurrentStatus()==SimbadLookupReply::SimbadLookupFinished)
	{
		simbadResults = simbadReply->getResults();
		ui->completionLabel->appendValues(simbadResults.keys());
		// Update push button enabled state
		ui->pushButtonGotoSearchSkyObject->setEnabled(!ui->completionLabel->isEmpty());
	}

	if (simbadReply->getCurrentStatus()!=SimbadLookupReply::SimbadLookupQuerying)
	{
		disconnect(simbadReply, SIGNAL(statusChanged()), this, SLOT(onSimbadStatusChanged()));
		delete simbadReply;
		simbadReply=NULL;

		// Update push button enabled state
		ui->pushButtonGotoSearchSkyObject->setEnabled(!ui->completionLabel->isEmpty());
	}
}

void SearchDialog::greekLetterClicked()
{
	QPushButton *sender = reinterpret_cast<QPushButton *>(this->sender());
	QLineEdit* sso = ui->lineEditSearchSkyObject;
	if (sender) {
		if (flagHasSelectedText)
		{
			sso->setText(sender->text());
			flagHasSelectedText = false;
		}
		else
			sso->setText(sso->text() + sender->text());
	}
	sso->setFocus();
}

void SearchDialog::gotoObject()
{
	QString name = ui->completionLabel->getSelected();

	if (name.isEmpty())
		return;

	StelMovementMgr* mvmgr = GETSTELMODULE(StelMovementMgr);
	if (simbadResults.contains(name))
	{
		close();
		Vec3d pos = simbadResults[name];
		objectMgr->unSelect();
		mvmgr->moveToJ2000(pos, mvmgr->getAutoMoveDuration());
		ui->lineEditSearchSkyObject->clear();
		ui->completionLabel->clearValues();
	}
	else if (objectMgr->findAndSelectI18n(name))
	{
		const QList<StelObjectP> newSelected = objectMgr->getSelectedObject();
		if (!newSelected.empty())
		{
			close();
			ui->lineEditSearchSkyObject->clear();
			ui->completionLabel->clearValues();
			// Can't point to home planet
			if (newSelected[0]->getEnglishName()!=StelApp::getInstance().getCore()->getCurrentLocation().name)
			{
				mvmgr->moveToObject(newSelected[0], mvmgr->getAutoMoveDuration());
				mvmgr->setFlagTracking(true);
			}
			else
			{
				GETSTELMODULE(StelObjectMgr)->unSelect();
			}
		}
	}
	simbadResults.clear();
}

bool SearchDialog::eventFilter(QObject*, QEvent *event)
{
	if (event->type() == QEvent::KeyRelease)
	{
		QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

		if (keyEvent->key() == Qt::Key_Tab || keyEvent->key() == Qt::Key_Down)
		{
			ui->completionLabel->selectNext();
			event->accept();
			return true;
		}
		if (keyEvent->key() == Qt::Key_Up)
		{
			ui->completionLabel->selectPrevious();
			event->accept();
			return true;
		}
	}

	return false;
}

QString SearchDialog::substituteGreek(const QString& keyString)
{
	if (!keyString.contains(' '))
		return getGreekLetterByName(keyString);
	else
	{
		QStringList nameComponents = keyString.split(" ", QString::SkipEmptyParts);
		if(!nameComponents.empty())
			nameComponents[0] = getGreekLetterByName(nameComponents[0]);
		return nameComponents.join(" ");
	}
}

QString SearchDialog::getGreekLetterByName(const QString& potentialGreekLetterName)
{
	if(greekLetters.contains(potentialGreekLetterName))
		return greekLetters[potentialGreekLetterName.toLower()];

	// There can be indices (e.g. "α1 Cen" instead of "α Cen A"), so strip
	// any trailing digit.
	int lastCharacterIndex = potentialGreekLetterName.length()-1;
	if(potentialGreekLetterName.at(lastCharacterIndex).isDigit())
	{
		QChar digit = potentialGreekLetterName.at(lastCharacterIndex);
		QString name = potentialGreekLetterName.left(lastCharacterIndex);
		if(greekLetters.contains(name))
			return greekLetters[name.toLower()] + digit;
	}

	return potentialGreekLetterName;
}

void SearchDialog::populateMirrorList()
{
	Q_ASSERT(ui);
	Q_ASSERT(ui->useSimbadMirrorComboBox);

	QComboBox* mirrors = ui->useSimbadMirrorComboBox;
	//Save the current selection to be restored later
	mirrors->blockSignals(true);
	int index = mirrors->currentIndex();
	QVariant selectedMirrorId = mirrors->itemData(index);
	mirrors->clear();
	//For each mirror, display the localized description and store the URL as user data.
	mirrors->addItem(q_("SIMBAD - University of Strasbourg"), "http://simbad.u-strasbg.fr/");
	mirrors->addItem(q_("SIMBAD - Harvard University"), "http://simbad.harvard.edu/");

	//Restore the selection
	index = mirrors->findData(selectedMirrorId, Qt::UserRole, Qt::MatchCaseSensitive);
	mirrors->setCurrentIndex(index);
	mirrors->model()->sort(0);
	mirrors->blockSignals(false);
}

// Called when the mirror is changed by hand
void SearchDialog::mirrorBoxChanged(const QString&)
{
	int index = ui->useSimbadMirrorComboBox->currentIndex();
	if (index < 0)
		useMirror = QString();//As returned by QComboBox::currentText()
	else
		useMirror = ui->useSimbadMirrorComboBox->itemData(index).toString();

	QSettings* conf = StelApp::getInstance().getSettings();
	Q_ASSERT(conf);
	conf->setValue("search/use_simbad_mirror_url", useMirror);
}
